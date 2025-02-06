import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the absolute path to the Xacro file
    package_path = get_package_share_directory('remote_node')
    urdf_output_path = os.path.join(package_path, 'urdf', 'swiftpro.urdf')
    
    print(f"Package path: {package_path}")
    print(f"URDF output path: {urdf_output_path}")
    
    # Robot State Publisher (does not start immediately)
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_output_path])}],
        output='screen'
    )

    
    # Add Static Transform Publisher to publish a transform from map to Base
    static_transform_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', '1','map', 'Base'],
        output='screen'
    )

    # Launch RViz to visualize the robot
    rviz_config_file = os.path.join(package_path, 'launch', 'swiftpro.rviz')  # Make sure this path is correct
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return launch.LaunchDescription([
        robot_state_publisher,
        static_transform_publisher,
        rviz
    ])
