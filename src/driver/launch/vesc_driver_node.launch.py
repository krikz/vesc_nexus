import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    conf = [PathJoinSubstitution([
                FindPackageShare('vesc_driver'), 'config', 'single_can_quad.yaml'])
            ]
    print(conf)
    return LaunchDescription([
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=conf,
        ),

    ])