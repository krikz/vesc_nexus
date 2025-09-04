from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FileContent

def generate_launch_description():
    urdf = FileContent(PathJoinSubstitution([FindPackageShare('vesc_nexus', 'robot.urdf.xacro')]))
    controller_config = os.path.join(get_package_share_directory('vesc_nexus'), 'config', 'robot_controller.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[controller_config,{'robot_description': urdf}],
            arguments=[urdf]),
        Node(
            package='controller_manager',
            executable='spawner',
            parameters=[controller_config],
            arguments=["diff_drive_controller", "-c", "/controller_manager", "-t", "diff_drive_controller/DiffDriveController"],
        ),
    ])