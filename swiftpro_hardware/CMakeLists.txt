cmake_minimum_required(VERSION 3.8)
project(swiftpro_hardware)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)


# Add executable
add_executable(swiftpro_hardware src/swiftpro_hardware.cpp)

# Link libraries
ament_target_dependencies(swiftpro_hardware
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS swiftpro_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()

