import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include the robot description launch file (visualization only)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('buraq_description'),
                'launch',
                'display.launch.py'
            ])
        ])
    )
    
    # Include the controller launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('buraq_control'),
                'launch',
                'buraq_control.launch.py'
            ])
        ])
    )
    
    # Create and return launch description
    return LaunchDescription([
        description_launch,
        control_launch
    ])
