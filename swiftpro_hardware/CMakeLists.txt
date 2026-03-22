cmake_minimum_required(VERSION 3.8)
project(swiftpro_hardware)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swiftpro_resources REQUIRED)

# Build the uarm library
add_subdirectory(lib)

# Add executable
add_executable(swiftpro_hardware src/swiftpro_hardware.cpp)

# Include directories for uarm library
target_include_directories(swiftpro_hardware
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
)

# Link libraries
ament_target_dependencies(swiftpro_hardware
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  swiftpro_resources
)

target_link_libraries(swiftpro_hardware
  uarm
)

# Install the executable
install(TARGETS swiftpro_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()
