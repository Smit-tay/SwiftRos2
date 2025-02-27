cmake_minimum_required(VERSION 3.8)
project(swiftpro_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Add executable
add_executable(swiftpro_control src/swiftpro_control.cpp)

# Link libraries
ament_target_dependencies(swiftpro_control
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ros2_control
  ros2_controllers
  robot_state_publisher
)

# Install the executable
install(TARGETS swiftpro_control
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()

