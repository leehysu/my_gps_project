#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" // For setRPY
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For toMsg() if needed, or direct assignment
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> // For M_PI, sin, cos, radians
#include <filesystem> // For std::filesystem::exists
#include <algorithm> // For std::min, std::max // NEW INCLUDE

/* 보간 있는 버전 ---------------------------- */

// Define a struct to hold interpolated points
struct PointCov {
    double x;
    double y;
};

static constexpr double R_EARTH = 6378137.0; // Earth's radius in meters (not directly used for CSV X,Y, but kept for consistency if latlon_to_local was used)

/* 인접 점 필터링 (거리 ≥ min_distance) */
std::vector<PointCov>
filter_points_by_distance(const std::vector<PointCov>& pts, double min_d)
{
    if (pts.empty()) return pts;
    std::vector<PointCov> filtered; filtered.push_back(pts.front());
    for (size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i].x - filtered.back().x;
        const double dy = pts[i].y - filtered.back().y;
        if (std::hypot(dx, dy) >= min_d) filtered.push_back(pts[i]);
    }
    return filtered;
}

/* 자연 1D 큐빅 스플라인 보간 (cov까지 선형 보간) */
std::vector<PointCov>
compute_cubic_spline(const std::vector<PointCov>& pts,
                     double target_spacing,
                     rclcpp::Logger logger) // Pass logger for ROS2 logging
{
    const int n = pts.size();
    if (n < 2) {
        RCLCPP_ERROR(logger, "보간할 점이 부족합니다.");
        return {};
    }

    /* 누적 거리 s */
    std::vector<double> s(n, 0.0);
    for (int i = 1; i < n; ++i)
        s[i] = s[i-1] +
               std::hypot(pts[i].x - pts[i-1].x,
                          pts[i].y - pts[i-1].y);

    const double total_len = s.back();
    if (total_len == 0.0) {
        RCLCPP_ERROR(logger, "전체 길이가 0입니다.");
        return {};
    }

    /* 좌표 및 2차 미분 계산 (x, y 각각) ---------------------------- */
    auto spline_second = [&](const std::vector<double>& v)
    {
        std::vector<double> y2(n, 0.0), u(n, 0.0);
        for (int i = 1; i < n-1; ++i) {
            double sig = (s[i] - s[i-1]) / (s[i+1] - s[i-1]);
            double p   = sig * y2[i-1] + 2.0;
            y2[i]      = (sig - 1.0) / p;
            double dv  = (v[i+1]-v[i])/(s[i+1]-s[i])
                       - (v[i]-v[i-1])/(s[i]-s[i-1]);
            u[i] = (6.0 * dv / (s[i+1]-s[i-1]) - sig * u[i-1]) / p;
        }
        y2[n-1] = 0.0;
        for (int k = n-2; k >= 0; --k)
            y2[k] = y2[k]*y2[k+1] + u[k];
        return y2;
    };

    std::vector<double> xs(n), ys(n); // Removed covs(n)
    for (int i = 0; i < n; ++i) {
        xs[i]   = pts[i].x;
        ys[i]   = pts[i].y;
        // Removed covs[i] = pts[i].cov;
    }
    const auto y2x = spline_second(xs);
    const auto y2y = spline_second(ys);

    /* 재샘플링 ---------------------------------------------- */
    std::vector<PointCov> out;
    for (double si = 0.0; si <= total_len; si += target_spacing) {
        // Ensure the last point is included even if target_spacing doesn't perfectly align
        double current_si = si;
        if (si > total_len) {
            current_si = total_len;
        }

        int k = 0;
        while (k < n-1 && s[k+1] < current_si) ++k;
        if (k >= n-1) k = n-2; // Handle edge case for last segment

        const double h = s[k+1] - s[k];
        if (h == 0.0) { // Avoid division by zero if two consecutive points are identical
            if (k + 1 < n) { // If not the very last point, use the next segment
                k++;
                if (k >= n-1) k = n-2; // Re-adjust if k becomes out of bounds
                const double h_new = s[k+1] - s[k];
                if (h_new == 0.0) { // If still zero, skip this point
                    continue;
                }
            } else { // If it's the last point and h is zero, just add it and break
                out.push_back(pts.back());
                break;
            }
        }

        const double a = (s[k+1] - current_si) / h;
        const double b = (current_si - s[k]) / h;

        const double xi = a*xs[k] + b*xs[k+1]
                        + ((a*a*a - a)*y2x[k] + (b*b*b - b)*y2x[k+1]) * h*h/6.0;
        const double yi = a*ys[k] + b*ys[k+1]
                        + ((a*a*a - a)*y2y[k] + (b*b*b - b)*y2y[k+1]) * h*h/6.0;
        // Removed const double ci = a*covs[k] + b*covs[k+1];
        out.push_back({xi, yi}); // Modified
        
        if (current_si == total_len) break;
    }
    // Ensure the very last point is always included
    if (out.empty() ||
        std::hypot(out.back().x - pts.back().x, out.back().y - pts.back().y) > 1e-6) // Check if last point is already there
    {
        out.push_back(pts.back());
    }

    return out;
}

// Helper function to convert Euler angles to a tf2::Quaternion
tf2::Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

class TfGpsCsvNode : public rclcpp::Node
{
public:
    TfGpsCsvNode() : Node("tf_gps_csv_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("csv_file_path", "/home/hannibal/Mandol_ws/data/processed/300_bag_15hz.csv");
        this->declare_parameter<double>("target_spacing", 0.3); // 보간된 경로의 점들 사이의 원하는 간격(미터)
        this->declare_parameter<double>("min_distance", 0.1);   // 필터링 단계에서 인접한 원본 점들 사이의 최소 거리(미터)입니다. 이 거리보다 가까운 점은 필터링됩니다.
        
        // Get parameters
        target_spacing_ = this->get_parameter("target_spacing").as_double(); 
        min_distance_ = this->get_parameter("min_distance").as_double();    

        // Initialize TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Initialize data storage
        origin_x_ = std::nullopt; // Use std::nullopt for optional doubles
        origin_y_ = std::nullopt;
        current_yaw_radians_ = 0.0;

        // Create Subscriptions
        f9r_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/f9r_utm", 10, std::bind(&TfGpsCsvNode::f9r_callback, this, std::placeholders::_1));
        f9p_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/f9p_utm", 10, std::bind(&TfGpsCsvNode::f9p_callback, this, std::placeholders::_1));
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/azimuth_angle", 10, std::bind(&TfGpsCsvNode::yaw_callback, this, std::placeholders::_1));

        // Create Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/csv_path", 10);
        yaw_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/azimuth_angle_text", 10);

        // Load CSV path once
        load_csv_path();

        // Timer to continuously publish CSV path
        // 변경 (50=0.05sec = 20 Hz)
        path_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&TfGpsCsvNode::publish_path_callback, this));
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::optional<double> origin_x_; // Using std::optional for nullable doubles
    std::optional<double> origin_y_;
    double current_yaw_radians_;
    nav_msgs::msg::Path path_msg_;

    // NEW MEMBER VARIABLES FOR INTERPOLATION
    double target_spacing_;
    double min_distance_;
    std::vector<PointCov> raw_csv_points_;
    std::vector<PointCov> interpolated_csv_points_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr f9r_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr f9p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_marker_pub_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;

    void load_csv_path()
    {
        std::string csv_file = this->get_parameter("csv_file_path").as_string();
        if (!std::filesystem::exists(csv_file))
        {
            RCLCPP_ERROR(this->get_logger(), "CSV file not found at: %s", csv_file.c_str());
            return;
        }

        std::ifstream file(csv_file);
        std::string line;
        int i = 0;
        raw_csv_points_.clear(); // Clear any previous data

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> seglist;

            while(std::getline(ss, segment, ','))
            {
                seglist.push_back(segment);
            }

            try
            {
                if (seglist.size() < 2) {
                    RCLCPP_WARN(this->get_logger(), "Skipping row %d (not enough columns): %s", i, line.c_str());
                    i++;
                    continue;
                }
                double x = std::stod(seglist[0]);
                double y = std::stod(seglist[1]);

                // Store raw points
                raw_csv_points_.push_back({x, y});
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Could not parse row %d (likely header): %s. Error: %s", i, line.c_str(), e.what());
            }
            i++;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu raw CSV points.", raw_csv_points_.size());

        if (raw_csv_points_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid points loaded from CSV.");
            return;
        }

        // Apply filtering and interpolation
        auto filtered_points = filter_points_by_distance(raw_csv_points_, min_distance_);
        interpolated_csv_points_ = compute_cubic_spline(filtered_points, target_spacing_, this->get_logger());

        RCLCPP_INFO(this->get_logger(), "Interpolated CSV path with %zu poses.", interpolated_csv_points_.size());

        // Populate path_msg_ from interpolated points
        path_msg_.header.frame_id = "csv"; // Stamp will be updated in publish_path_callback
        path_msg_.poses.clear(); // Clear any previous poses

        // Set origin on the first valid interpolated data point
        if (!interpolated_csv_points_.empty() && !origin_x_.has_value())
        {
            origin_x_ = interpolated_csv_points_[0].x;
            origin_y_ = interpolated_csv_points_[0].y;
            RCLCPP_INFO(this->get_logger(), "CSV origin set to: x=%.2f, y=%.2f", origin_x_.value(), origin_y_.value());
        }

        for (const auto& p : interpolated_csv_points_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "csv"; // Stamp will be updated in publish_path_callback
            pose.pose.position.x = p.x - origin_x_.value();
            pose.pose.position.y = p.y - origin_y_.value();
            pose.pose.position.z = 0.0; // Assuming 2D path
            path_msg_.poses.push_back(pose);
        }
    }

    void publish_path_callback()
    {
        if (!path_msg_.poses.empty())
        {
            path_msg_.header.stamp = this->get_clock()->now();
            for (auto& pose : path_msg_.poses)
            {
                pose.header.stamp = path_msg_.header.stamp;
            }
            path_pub_->publish(path_msg_);
            // RCLCPP_INFO(this->get_logger(), "Published CSV path with %zu poses.", path_msg_.poses.size()); // Too verbose
        }
    }

    void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Yaw from North (degrees) to ROS standard yaw (radians from East)
        // ROS yaw: 0 is +X (East), increases counter-clockwise.
        // North is +Y. So, 90 degrees from North (clockwise) is East.
        // 90 - msg.data converts North-based clockwise degrees to East-based counter-clockwise degrees.
        current_yaw_radians_ = (90.0 - msg->data) * M_PI / 180.0;

        // --- NEW FEATURE: Publish yaw as a text marker ---
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "f9r"; // Anchor the marker to the f9r frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "azimuth_angle_text";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position the marker to the right of the f9r frame origin
        // In ROS, Y is typically left, so a negative Y is to the right.
        marker.pose.position.x = -1.0;
        marker.pose.position.y = -0.5; // 50cm to the right
        marker.pose.position.z = 0.5;  // 50cm up

        // Text properties
        marker.scale.z = 0.4; // Text height
        marker.color.a = 1.0; // Alpha (must be non-zero)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0; // White
        marker.text = "/azimuth_angle " + std::to_string(msg->data) + "°";

        yaw_marker_pub_->publish(marker);
    }

    void f9r_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!origin_x_.has_value())
        {
            return;
        }
        broadcast_transform(msg, "f9r");
    }

    void f9p_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!origin_x_.has_value())
        {
            return;
        }
        broadcast_transform(msg, "f9p");
    }

    void broadcast_transform(const geometry_msgs::msg::PointStamped::SharedPtr msg, const std::string& child_frame_id)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "csv"; // Parent frame for the GPS sensors
        t.child_frame_id = child_frame_id; // Child frame (f9r or f9p)

        // Translate the GPS point relative to the CSV origin
        t.transform.translation.x = msg->point.x - origin_x_.value();
        t.transform.translation.y = msg->point.y - origin_y_.value();
        t.transform.translation.z = 0.0; // Assuming 2D for now

        // Apply the current yaw to the transform
        tf2::Quaternion q = quaternion_from_euler(0, 0, current_yaw_radians_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfGpsCsvNode>());
    rclcpp::shutdown();
    return 0;
}
