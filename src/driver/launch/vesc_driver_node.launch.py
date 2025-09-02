import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo


def generate_launch_description():
    # Создаем подстановку для пути к YAML-файлу
    config_path = PathJoinSubstitution([
        FindPackageShare('vesc_driver'), 'config', 'single_can_quad.yaml'
    ])

    # Выводим путь к конфигурации в лог
    return LaunchDescription([
        # Логируем путь к файлу
        LogInfo(msg=["Configuration file path: ", config_path]),

        # Запускаем ноду с параметрами
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[config_path],
        ),
    ])