import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Arguments
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')

    # Obtains andino_description's share directory path.
    pkg_andino_description = get_package_share_directory('buraq_description')

    # Obtain urdf from xacro files.
    doc = xacro.process_file(os.path.join(pkg_andino_description, 'urdf', '4wd.urdf.xacro'))
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        rsp_argument,
        rsp,
    ])
