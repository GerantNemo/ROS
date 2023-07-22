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
            package='sub_guidage_c',
            namespace='sub_guidage_c',
            executable='guidage_sonar3',
            #parameters = [config],
            name='guidage_sonar3',
            remappings=[
                ('/sub_guidage_c/SousMarin/Sonar/Pos_Init', '/SousMarin/Sonar/Pos_Init'),
                ('/sub_guidage_c/SousMarin/Sonar/Pos_Cible', '/SousMarin/Sonar/Pos_Cible'),
                ('/sub_guidage_c/SousMarin/Sonar/Angle', '/SousMarin/Sonar/Angle'),
                ('/sub_guidage_c/SousMarin/Sonar/Requete', '/SousMarin/Sonar/Requete'),
                ('/sub_guidage_c/SousMarin/command/mode', '/SousMarin/command/mode'),
                ('/sub_guidage_c/SousMarin/command/pos_vel_pos', '/SousMarin/command/pos_vel_pos'),
                ('/sub_guidage_c/SousMarin/command/pos', '/SousMarin/command/pos'),
                ('/sub_guidage_c/SousMarin/command/bloquage', '/SousMarin/command/bloquage'),
                ]
            )
    
    ld.add_action(guidage_sonar3)

    return ld