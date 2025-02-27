cmake_minimum_required(VERSION 3.8)
project(swiftpro_kinematics)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(swiftpro_kinematics src/swiftpro_kinematics.cpp)

ament_target_dependencies(swiftpro_kinematics 
    rclcpp 
    geometry_msgs
    sensor_msgs )

install(TARGETS swiftpro_kinematics DESTINATION share/${PROJECT_NAME})

# Export package dependencies
ament_package()
