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
    
    guidage_sonar3 = Node(
            package='sub_etat_py',
            namespace='sub_etat_py',
            executable='etat_node',
            #parameters = [config],
            name='etat_node',
            remappings=[
                ('/sub_etat_py/SousMarin/measured/Depth', '/SousMarin/measured/Depth'),
                ('/sub_etat_py/model/SousMarin/odometry', '/model/SousMarin/odometry'),
                ]
            )
    
    ld.add_action(guidage_sonar3)

    return ld