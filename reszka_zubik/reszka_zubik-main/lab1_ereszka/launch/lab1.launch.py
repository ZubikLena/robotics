from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

def generate_launch_description():
    '''
    function launching turtlesim and publisher nodes
    
    Parameters:
    -----
    package: string
        used package name
    executable: string
        node
    name: string
        node's name
    parameters: dict
        node's parameters
    prefix: string
        emulating gnome terminal
    '''
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='lab1_ereszka',
            executable='publisher',
            name='publisher',
            prefix='gnome-terminal --',
            parameters=[{
                'left': 'a',
                'right': 's',
                'up': 'w',
                'down': 'z'
            }]
        )
    ])