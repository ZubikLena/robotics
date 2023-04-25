from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='py_motion',
            executable='py_pub',
            name='my_pub',
            prefix= ['gnome-terminal --disable-factory --'],
            parameters = [
                {'my_parameter': 'gdpl'}
            ]
        )
    ])
