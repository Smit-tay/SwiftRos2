cmake_minimum_required(VERSION 3.14)
project(swiftpro_hardware)

# -----------------------------------------------------------------------------
# Compile settings
# -----------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# ROS2 dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swiftpro_resources REQUIRED)

# -----------------------------------------------------------------------------
# libswiftpro — vendored at SwiftRos2/third_party/libswiftpro
# Built in-tree via add_subdirectory. Modern CMake usage requirements
# propagate include paths automatically through the swiftpro target.
# -----------------------------------------------------------------------------
# Skip libswiftpro examples — they're for standalone use, not consumers.
set(BUILD_EXAMPLES OFF CACHE BOOL "Build libswiftpro examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libswiftpro
                 ${CMAKE_CURRENT_BINARY_DIR}/libswiftpro)

install(TARGETS swiftpro
  LIBRARY DESTINATION lib
)

# -----------------------------------------------------------------------------
# Hardware node
# -----------------------------------------------------------------------------
add_executable(swiftpro_hardware src/swiftpro_hardware.cpp)

ament_target_dependencies(swiftpro_hardware
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  swiftpro_resources
)

target_link_libraries(swiftpro_hardware
  swiftpro
)

install(TARGETS swiftpro_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
ament_package()
