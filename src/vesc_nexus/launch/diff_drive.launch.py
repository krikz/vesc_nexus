from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('vesc_nexus'), 'urdf', 'robot.urdf.xacro')
    controller_config = os.path.join(get_package_share_directory('vesc_nexus'), 'config', 'robot_controller.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config, {'robot_description': Command(['xacro ', urdf_path])}]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller']
        ),
    ])