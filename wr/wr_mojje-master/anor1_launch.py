
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='py_motion',
            executable='py_pub',
            name='pub',
            parameters = [
                {'my_parameter': 'gdpl'}
            ],
            prefix='gnome-terminal --disable-factory --',
            #remappings=[
            #    ('/output/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
            #]
        )
    ])
