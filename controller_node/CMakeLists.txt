cmake_minimum_required(VERSION 3.8)
project(controller_node)

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
add_executable(controller_node src/swiftpro_controller.cpp)

# Link libraries
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ros2_control
  ros2_controllers
  robot_state_publisher
)

# Install the executable
install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()

