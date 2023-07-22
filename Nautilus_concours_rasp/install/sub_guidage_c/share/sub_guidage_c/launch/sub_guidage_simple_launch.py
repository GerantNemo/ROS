import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    #config = os.path.join(
    #    get_package_share_directory('sub_guidage_c'),
    #    'config',
    #    'params.yaml'
    #    )
    
    guidage_simple = Node(
            package='sub_guidage_c',
            namespace='sub_guidage_c',
            executable='guidage_simple',
            #parameters = [config],
            name='guidage_simple',
            remappings=[
                ('/sub_guidage_c/SousMarin/command/mode', '/SousMarin/command/mode'),
                ('/sub_guidage_c/SousMarin/command/pos_vel_pos', '/SousMarin/command/pos_vel_pos')
                ]
            )
    
    ld.add_action(guidage_simple)

    return ld