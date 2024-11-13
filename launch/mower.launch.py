import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler


def generate_launch_description():

    # Get the package name
    package_name='mower_bot' 

    # Launches robot state publisher configured for real-time use.
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Launches the rplidar driver node and publishes the data to the /scan topic
    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar.launch.py'
                )])
    )

    # Launches twist mux node and remaps the output to our controller input
    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    # )

    # Get our robot's description from robot state publisher
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # Get the configuration files for our controller 
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    
    # Launches the controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Launches the differential drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Launches the joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch!
    return LaunchDescription([
        rsp,
        rplidar,
        #twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
