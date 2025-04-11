import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui')
    )
    
    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("buraq_description"), "urdf", "buraq.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Configure the nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    
    # URDF visualizer
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=LaunchConfiguration("use_gui", default="false"),
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=LaunchConfiguration("use_gui", default="true"),
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("buraq_description"), "config", "buraq.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    nodes = [
        robot_state_pub_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
