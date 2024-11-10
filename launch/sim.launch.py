import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction


def generate_launch_description():

    # Package name
    package_name='mower_bot' 

    # Launch configuration
    headless = LaunchConfiguration('headless')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Decides if the simulation is visualized')

    # Launch Robot State Publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    # Launch Joystick Tele Operation
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    
    # Setup the world
    world_path = os.path.join(get_package_share_directory(package_name),'worlds/box.world')
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Launch the gazebo server to initialize the simulation
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Launch the gazebo client to visualize the simulation only if headless is declared as False
    gazebo_client = GroupAction(
        condition=IfCondition(PythonExpression(['not ', headless])),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items())]
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    # Launch Rviz with pre-made view
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'mower.rviz'),
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    # Launch Differential Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Launch the Joint State Broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch Simultaneous Localization and Mapping
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','slam.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Work in progress - no idea why it does not work
    # navigation = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([os.path.join(
    #                     get_package_share_directory(package_name),'launch','nav.launch.py'
    #                 )])
    # )

    # Launch them all!
    return LaunchDescription([
        # Declare world path, if not treated as a launch config
        declare_world_cmd,
        declare_headless,

        # Launch the nodes
        rviz,
        rsp,
        joystick,
        twist_mux,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        slam,
        #navigation,
    ])