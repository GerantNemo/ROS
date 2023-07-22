import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    sonar = Node(
        package='sub_sonar',
        namespace='sub_sonar',
        executable='sonar_scan',
        name='sonar',
        remappings=[
            ('/sub_sonar/SousMarin/Sonar/Pos_Init', '/SousMarin/Sonar/Pos_Init'),
            ('/sub_sonar/SousMarin/Sonar/Pos_Cible', '/SousMarin/Sonar/Pos_Cible'),
            ('/sub_sonar/SousMarin/Sonar/Requete', '/SousMarin/Sonar/Requete')
        ]
    )

    ld.add_action(sonar)

    return ld