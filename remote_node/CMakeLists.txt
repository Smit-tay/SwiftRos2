cmake_minimum_required(VERSION 3.8)
project(remote_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(remote_node src/uarm_remote.cpp)

# Link libraries
ament_target_dependencies(remote_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS remote_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# Install the URDF/Xacro files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Export package dependencies
ament_package()

