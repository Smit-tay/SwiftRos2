# swiftpro_kinematics/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(swiftpro_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swiftpro_resources REQUIRED)

add_executable(swiftpro_kinematics src/swiftpro_kinematics.cpp)

target_include_directories(swiftpro_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(swiftpro_kinematics
  geometry_msgs
  rclcpp
  sensor_msgs
  swiftpro_resources
)

ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)
 
install(TARGETS swiftpro_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
