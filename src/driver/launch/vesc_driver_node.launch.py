import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'config',
        'single_can_quad.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
            ),
        Node(
            package='vesc_driver',
            executable='sfsdfsdfdsfdsfdsfdfsd',
            name='zxczczczcc',
            parameters=[LaunchConfiguration("config")]
        ),

    ])