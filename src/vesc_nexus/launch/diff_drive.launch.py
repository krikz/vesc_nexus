# launch/diff_drive.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Путь к URDF
    urdf_path = FindPackageShare('vesc_nexus').find('vesc_nexus') + '/urdf/robot.urdf.xacro'

    # Путь к конфигу
    controller_config = os.path.join(
        get_package_share_directory('vesc_nexus'),
        'config',
        'robot_controller.yaml'
    )

    return LaunchDescription([
        # Публикуем URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro', urdf_path])}],
            output='screen'
        ),

        # Запускаем controller_manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='screen',
            emulate_tty=True
        ),

        # Спавним контроллер
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
            emulate_tty=True
        ),
    ])