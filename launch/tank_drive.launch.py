from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{package_name}',
            executable='{package_name}_node',
            name='vesc_nexus_node',
            parameters=[{'can_interfaces': [
                {{'name': 'can0', 'baudrate': 500000, 'vesc_ids': [1, 2]}},
                {{'name': 'can1', 'baudrate': 500000, 'vesc_ids': [3, 4]}}
            ]}],
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        )
    ])
