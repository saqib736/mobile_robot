import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    controller_params_file = os.path.join(get_package_share_directory("buraq_bringup"),'config','controllers.yaml')

    # We need the robot description to be passed to the controller_manager
    # So it can check the ros2_control parameters.
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],
        remappings=[
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'), # Used if use_stamped_vel param is true
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'), # Used if use_stamped_vel param is false
            ('/diff_drive_controller/cmd_vel_out', '/cmd_vel_out'), # Used if publish_limited_velocity param is true
            ('/diff_drive_controller/odom', '/odom'),
        ],
        output="both",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        remappings=[('/imu_sensor_broadcaster/imu', '/imu'),]
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Delay start of robot_controller after joint_state_broadcaster
    delay_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    delay_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )
    
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner,
        delay_imu_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)

