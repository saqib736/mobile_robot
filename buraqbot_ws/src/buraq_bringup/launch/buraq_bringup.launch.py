import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

pkg_buraq_control = get_package_share_directory('buraq_bringup')
pkg_buraq_description = get_package_share_directory('buraq_description')

def generate_launch_description():

    include_buraq_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buraq_description, 'launch', 'description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )

    include_buraq_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buraq_control, 'launch', 'buraq_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    buraq_control_timer = TimerAction(period=5.0, actions=[include_buraq_control])

    return LaunchDescription([
        include_buraq_description,
        buraq_control_timer,
    ])
