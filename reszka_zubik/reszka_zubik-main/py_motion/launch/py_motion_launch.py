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
            prefix= ['gnome-terminal --'],
            parameters = [
                {'my_parameter_up': 'g',
                'my_parameter_down': 'd',
                'my_parameter_right': 'p',
                'my_parameter_left': 'l',
                'my_parameter_stop': 's',}
            ]
        )
    ])
