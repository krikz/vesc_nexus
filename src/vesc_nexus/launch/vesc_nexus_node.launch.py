from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

import rclpy.logging

# Создаем логгер для вывода отладочной информации
logger = rclpy.logging.get_logger("vesc_nexus.launch")

def launch_setup(context):
    # Создаем подстановку для пути к YAML-файлу
    config_path_subst = PathJoinSubstitution([
        FindPackageShare('vesc_nexus'), 'config', 'vesc_nexus_config.yaml'
    ])

    # Выполняем подстановку и получаем фактический путь
    config_path_str = config_path_subst.perform(context)
    logger.info(f"Resolved configuration file path: {config_path_str}")

    # Читаем содержимое файла
    with open(config_path_str, 'r') as file:
        config_content = file.read()
    
    # Логируем содержимое файла
    logger.info(f"Configuration file content:\n{config_content}")
    
    # Создаем описание параметров
    file_parameters = ParameterFile(
        param_file=config_path_subst,
        allow_substs=True
    )

    # Возвращаем ноду с параметрами
    return [
        Node(
            package='vesc_nexus',
            executable='vesc_can_driver_node',
            name='vesc_can_driver',
            parameters=[file_parameters],
            output='screen',
            emulate_tty=True,  # Красивый вывод (цвета, логи в реальном времени)
            respawn=True,      # Перезапуск при падении (опционально)
            respawn_delay=2    # Задержка перед перезапуском
        )
    ]

def generate_launch_description():
    # Объявляем аргументы (если нужны)
    declared_args = []

    # Возвращаем описание запуска
    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_setup)
    ])