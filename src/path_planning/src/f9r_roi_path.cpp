#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <vector>
#include <limits>
#include <cmath>
#include <memory>

#include "path_planning/gpu_roi.hpp"

class ROIPathPublisher : public rclcpp::Node {
public:
  ROIPathPublisher()
  : rclcpp::Node("f9r_roi_path_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    has_path_(false),
    last_min_idx_(0)
  {
    this->declare_parameter<double>("roi_arc_length", 3.8);
    this->declare_parameter<std::string>("target_frame", "f9r");
    this->declare_parameter<double>("timer_frequency", 20.0); // NEW PARAMETER
    roi_arc_length_ = this->get_parameter("roi_arc_length").as_double();
    target_frame_   = this->get_parameter("target_frame").as_string();
    timer_frequency_ = this->get_parameter("timer_frequency").as_double(); // GET PARAMETER

    pre_transformed_.reserve(2000);
    cumulative_arc_.reserve(2000);

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/csv_path", 1, std::bind(&ROIPathPublisher::pathCallback, this, std::placeholders::_1));

    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/f9r/fix", 1, std::bind(&ROIPathPublisher::fixCallback, this, std::placeholders::_1));

    roi_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/f9r_roi_path", 1);
    roi_end_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("/f9r_roi_end", 1);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_frequency_), std::bind(&ROIPathPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "[f9r] ROI arc=%.2f, frame=%s", roi_arc_length_, target_frame_.c_str());
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      has_path_ = false;
      pre_transformed_.clear();
      cumulative_arc_.clear();
      gpu_.reset();
      return;
    }
    const size_t N = msg->poses.size();
    pre_transformed_.resize(N);
    cumulative_arc_.clear();
    cumulative_arc_.reserve(N);

    double acc = 0.0;
    for (size_t i=0;i<N;++i){
      pre_transformed_[i].x = msg->poses[i].pose.position.x;
      pre_transformed_[i].y = msg->poses[i].pose.position.y;
      pre_transformed_[i].z = 0.0; // Assuming 2D path

      if (i==0){
        cumulative_arc_.push_back(0.0);
      } else {
        double dx = pre_transformed_[i].x - pre_transformed_[i-1].x;
        double dy = pre_transformed_[i].y - pre_transformed_[i-1].y;
        acc += std::hypot(dx, dy);
        cumulative_arc_.push_back(acc);
      }
    }
    has_path_ = true;
    last_min_idx_ = 0;
    RCLCPP_INFO(this->get_logger(), "[f9r] path updated: %zu points", N);
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr /*msg*/) {}

  void timerCallback() {
    if (!has_path_ || pre_transformed_.empty()) return;

    const size_t N = pre_transformed_.size();
    if (last_min_idx_ >= N) last_min_idx_ = 0;

    // Get robot's current position in the CSV frame
    geometry_msgs::msg::TransformStamped robot_to_csv_tf;
    try {
      robot_to_csv_tf = tf_buffer_.lookupTransform(
        "csv", target_frame_, // Transform from robot's frame to csv frame
        tf2::TimePointZero, tf2::Duration(std::chrono::milliseconds(100))); // Use a short timeout
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup (robot to csv) failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PointStamped robot_pos_in_target_frame;
    robot_pos_in_target_frame.header.frame_id = target_frame_;
    robot_pos_in_target_frame.header.stamp = this->get_clock()->now();
    robot_pos_in_target_frame.point.x = 0.0; // Robot is at origin of its own frame
    robot_pos_in_target_frame.point.y = 0.0;
    robot_pos_in_target_frame.point.z = 0.0;

    geometry_msgs::msg::PointStamped robot_pos_in_csv_frame;
    tf2::doTransform(robot_pos_in_target_frame, robot_pos_in_csv_frame, robot_to_csv_tf);

    const double robot_x = robot_pos_in_csv_frame.point.x;
    const double robot_y = robot_pos_in_csv_frame.point.y;

    auto find_min = [&](size_t i0, size_t i1, bool require_xpos)->int{
      double best = std::numeric_limits<double>::infinity();
      int best_i = -1;
      for (size_t i=i0; i<i1; ++i){
        const auto& p = pre_transformed_[i];
        // Calculate distance from robot's position in CSV frame
        double dx = p.x - robot_x;
        double dy = p.y - robot_y;
        double d2 = dx*dx + dy*dy;
        if (d2 < best){ best = d2; best_i = static_cast<int>(i); }
      }
      return best_i;
    };

    const size_t W = 300;
    size_t i0 = (last_min_idx_ > W) ? (last_min_idx_ - W) : 0;
    size_t i1 = std::min(N, last_min_idx_ + W);

    int min_idx = find_min(i0, i1, /*require_xpos=*/true);
    if (min_idx < 0) {
      min_idx = find_min(0, N, /*require_xpos=*/true);
      if (min_idx < 0) {
        min_idx = find_min(0, N, /*require_xpos=*/false);
      }
    }
    if (min_idx < 0) return;

    last_min_idx_ = static_cast<size_t>(min_idx);

    size_t end_idx = static_cast<size_t>(min_idx);
    while (end_idx < N &&
           (cumulative_arc_[end_idx] - cumulative_arc_[min_idx]) <= roi_arc_length_) {
      ++end_idx;
    }
    if (end_idx == 0 || end_idx <= static_cast<size_t>(min_idx)) return;
    if (end_idx > N) end_idx = N;

    // LINE_STRIP (빨강)
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "csv"; // Changed to csv frame
    line.header.stamp = this->get_clock()->now();
    line.ns = "roi_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;
    line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; line.color.a = 1.0;
    line.points.reserve(end_idx - static_cast<size_t>(min_idx));
    for (size_t i = static_cast<size_t>(min_idx); i < end_idx; ++i) {
      line.points.push_back(pre_transformed_[i]);
    }
    roi_path_pub_->publish(line);

    visualization_msgs::msg::Marker endp;
    endp.header = line.header; // Use same header as line marker (csv frame)
    endp.ns = "roi_end_marker";
    endp.id = 0;
    endp.type = visualization_msgs::msg::Marker::SPHERE;
    endp.action = visualization_msgs::msg::Marker::ADD;
    endp.pose.position = pre_transformed_[end_idx - 1];
    endp.scale.x = endp.scale.y = endp.scale.z = 0.3;
    endp.color.r = 1.0; endp.color.g = 0.0; endp.color.b = 0.0; endp.color.a = 1.0;
    roi_end_pub_->publish(endp);
  }

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_end_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<geometry_msgs::msg::Point> pre_transformed_;
  std::vector<double> cumulative_arc_;
  bool has_path_;
  double roi_arc_length_;
  std::string target_frame_;
  double timer_frequency_; // NEW MEMBER

  size_t last_min_idx_;

  path_planning::GPURoiHelper gpu_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROIPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
