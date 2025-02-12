cmake_minimum_required(VERSION 3.8)
project(kinematics_node)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(kinematics_node src/kinematics_node.cpp)

ament_target_dependencies(kinematics_node 
    rclcpp 
    geometry_msgs
    sensor_msgs )

install(TARGETS kinematics_node DESTINATION share/${PROJECT_NAME})

# Export package dependencies
ament_package()
