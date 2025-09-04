# launch/diff_drive.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Путь к URDF через xacro
    pkg_share = get_package_share_directory('vesc_nexus')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Конфиг контроллера
    controller_config = os.path.join(
        get_package_share_directory('vesc_nexus'),
        'config',
        'robot_controller.yaml'
    )

    return LaunchDescription([
        # Публикуем раскрытый URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', urdf_path]), value_type=str
                )
            }]
        ),

        # Запускаем spawner с явным указанием типа
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
                '--controller-type', 'diff_drive_controller/DiffDriveController'
            ],
            output='screen'
        ),
    ])