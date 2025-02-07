import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



# This file is designed to show the urdf representation of the UArm SwiftPro within rViz2.
# We want to do this because it's important to know that we are working with a reasonable
# representation of the physical robot, and that representation (xacro + STL FIles) works
# without using *any* of the nodes created by this project - so, this is an independent test.

def generate_launch_description():
    # Since the xacro and STL files are "part" or the remote node, the get installed
    # in the remote node's install path.  Get that install directory.
    package_path = get_package_share_directory('remote_node')
    # Now, form a path to the xacro generated urdf file.
    # os.path.join() is just a concatenator which understands OS specific path syntax.
    urdf_output_path = os.path.join(package_path, 'urdf', 'swiftpro.urdf')
    # Path to the RViz config file
    rviz_config_path = os.path.join(package_path, 'config', 'swiftpro.rviz')
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_output_path).read()}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_path]  # Load the RViz config file
        )
    ])
