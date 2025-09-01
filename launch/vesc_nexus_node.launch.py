from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = LaunchConfiguration('config', default=os.path.join(
        get_package_share_directory('{package_name}'),
        'config',
        'vesc_nexus_example.yaml'
    ))

    return LaunchDescription([
        DeclareLaunchArgument('config', description='Path to config file'),

        Node(
            package='{package_name}',
            executable='{package_name}_node',
            name='{package_name}_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
