cmake_minimum_required(VERSION 3.8)
project(cal_azimuth_angle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(azimuth_angle_calculator_node cal_azimuth_angle/azimuth_angle_calculator.cpp)
ament_target_dependencies(azimuth_angle_calculator_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  azimuth_angle_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
