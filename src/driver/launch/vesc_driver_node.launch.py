from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к папке config в пакете vesc_driver
    config_pkg_share = FindPackageShare('vesc_driver')

    # Аргумент 'config' с дефолтом — single_can_quad.yaml
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([config_pkg_share, 'config', 'single_can_quad.yaml']),
        description='Path to config file to load'
    )

    # Получаем значение аргумента
    config = LaunchConfiguration('config')

    return LaunchDescription([
        config_arg,
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])