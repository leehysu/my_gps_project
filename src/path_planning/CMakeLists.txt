cmake_minimum_required(VERSION 3.18)
project(path_planning LANGUAGES CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# GPU helper (경로 갱신 시에만 사용)
add_library(gpu_roi STATIC src/gpu_roi.cu)
set_target_properties(gpu_roi PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(gpu_roi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# f9p node
add_executable(f9p_roi_path src/f9p_roi_path.cpp)
target_include_directories(f9p_roi_path PRIVATE include)
ament_target_dependencies(f9p_roi_path
  rclcpp nav_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs sensor_msgs
)
target_link_libraries(f9p_roi_path gpu_roi)  # plain signature

# f9r node
add_executable(f9r_roi_path src/f9r_roi_path.cpp)
target_include_directories(f9r_roi_path PRIVATE include)
ament_target_dependencies(f9r_roi_path
  rclcpp nav_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs sensor_msgs
)
target_link_libraries(f9r_roi_path gpu_roi)  # plain signature

install(TARGETS
  gpu_roi
  f9p_roi_path
  f9r_roi_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
