cmake_minimum_required(VERSION 3.8)
project(swiftpro_hardware)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swiftpro_resources REQUIRED)

# -----------------------------------------------------------------------------
# uarm library
# -----------------------------------------------------------------------------
add_subdirectory(lib)

install(TARGETS uarm
  LIBRARY DESTINATION lib
)

# -----------------------------------------------------------------------------
# Hardware node
# -----------------------------------------------------------------------------
add_executable(swiftpro_hardware src/swiftpro_hardware.cpp)

target_include_directories(swiftpro_hardware
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
)

ament_target_dependencies(swiftpro_hardware
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  swiftpro_resources
)

target_link_libraries(swiftpro_hardware
  uarm
)

install(TARGETS swiftpro_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
ament_package()
