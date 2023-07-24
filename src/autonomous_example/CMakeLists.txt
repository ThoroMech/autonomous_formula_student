cmake_minimum_required(VERSION 3.8)
project(autonomous_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

# Add executable
add_executable(cone_id src/cone_id.cpp)
add_executable(steering_angle src/steering_angle.cpp)
add_executable(throttle_pos src/throttle_pos.cpp)
add_executable(car_controller src/car_controller.cpp)
ament_target_dependencies(cone_id rclcpp sensor_msgs std_msgs)
ament_target_dependencies(steering_angle rclcpp geometry_msgs)
ament_target_dependencies(throttle_pos rclcpp sensor_msgs std_msgs)
ament_target_dependencies(car_controller rclcpp std_msgs fs_msgs)

# Install executable
install(TARGETS 
  cone_id
  steering_angle
  throttle_pos
  car_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
