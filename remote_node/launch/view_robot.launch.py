import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the absolute path to the Xacro file
    package_path = get_package_share_directory('remote_node')
    xacro_file = os.path.join(package_path, 'urdf', 'swiftpro.xacro')
    urdf_output_path = '/tmp/swiftpro.urdf'  # Temporary URDF file

    # Generate URDF from Xacro
    generate_urdf = ExecuteProcess(
        cmd=['ros2', 'run', 'xacro', 'xacro', xacro_file, '-o', urdf_output_path],
        output='screen'
    )

    # Robot State Publisher (does not start immediately)
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_output_path])}],
        output='screen'
    )

    # Ensure Robot State Publisher starts only after Xacro finishes
    start_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_urdf,
            on_exit=[robot_state_publisher]
        )
    )

    return launch.LaunchDescription([
        generate_urdf,
        start_robot_state_publisher
    ])
