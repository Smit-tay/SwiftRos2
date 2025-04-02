import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_path = get_package_share_directory('swiftpro_resources')
    xacro_path = os.path.join(package_path, 'urdf', 'swiftpro.xacro')
    rviz_config_path = os.path.join(package_path, 'visualizer_config', 'swiftpro-dual-instance.rviz')

    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        # Static transform: world -> swiftpro_1/dummy_base
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_swiftpro_1",
            arguments=["0", "0", "0", "0", "0", "0", "1", "world", "swiftpro_1/dummy_base"],
            output="screen"
        ),

        # Static transform: world -> swiftpro_2/dummy_base (offset so they don’t overlap)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_swiftpro_2",
            arguments=["1", "0", "0", "0", "0", "0", "1", "world", "swiftpro_2/dummy_base"],
            output="screen"
        ),

        # swiftpro_1
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="swiftpro_1",
            parameters=[
                {"robot_description": robot_description},
                {"frame_prefix": "swiftpro_1/"},
                {"use_sim_time": False},
                {"publish_frequency": 50.0}
            ],
            remappings=[("/joint_states", "/swiftpro_1/joint_states")],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace="swiftpro_1",
            parameters=[{"robot_description": robot_description}],
            remappings=[("/joint_states", "/swiftpro_1/joint_states")],
            output="screen"
        ),

        # swiftpro_2
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="swiftpro_2",
            parameters=[
                {"robot_description": robot_description},
                {"frame_prefix": "swiftpro_2/"},
                {"use_sim_time": False},
                {"publish_frequency": 50.0}
            ],
            remappings=[("/joint_states", "/swiftpro_2/joint_states")],
            output="screen"
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace="swiftpro_2",
            parameters=[{"robot_description": robot_description}],
            remappings=[("/joint_states", "/swiftpro_2/joint_states")],
            output="screen"
        ),
        Node(
            package="swiftpro_kinematics",
            executable="swiftpro_kinematics",
            namespace="swiftpro_1",
            output="screen"
        ),
        Node(
            package="swiftpro_kinematics",
            executable="swiftpro_kinematics",
            namespace="swiftpro_2",
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_path],
            output="screen"
        )
    ])