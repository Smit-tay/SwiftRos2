cmake_minimum_required(VERSION 3.8)
project(swiftpro_kinematics)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)  # For tf2::Quaternion
find_package(tf2_ros REQUIRED)  # Must be here


add_executable(swiftpro_kinematics src/swiftpro_kinematics.cpp)

ament_target_dependencies(swiftpro_kinematics 
    rclcpp 
    tf2
    tf2_ros
    geometry_msgs
    sensor_msgs )

install(TARGETS swiftpro_kinematics DESTINATION lib/${PROJECT_NAME})

# Export package dependencies
ament_package()
