from launch import LaunchDescription
from launch_ros.actions import Node

ROBOTS = [
    ('cf_0', [0.8, 0.0, 0.5]),
    ('cf_1', [-0.8, 0.0, 0.5]),
    # ('cf_2', [-0.4, -0.6928, 0.5]),
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_controller',
            executable='hello_node',
            name=f'hello_node_{robot_name}',
            output='screen',
            parameters=[{'robot_name': robot_name, 'target': target}],        )

        for robot_name, target in ROBOTS
    ])