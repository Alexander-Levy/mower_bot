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

    # Launch configurations
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    
    # Launch Arguments
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Decides if the simulation is visualized')
    
    world_path = os.path.join(get_package_share_directory(package_name),'worlds/box.world')
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True'
    )
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
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'mower.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    remappings=[('/map', 'map'),
                                ('/tf', 'tf'),
                                ('/tf_static', 'tf_static'),
                                ('/goal_pose', 'goal_pose'),
                                ('/clicked_point', 'clicked_point'),
                                ('/initialpose', 'initialpose')])]
    )

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

    # Launch the navigation stack configured for coverage planning
    nav_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'coverage_params.yaml')
    navigation = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','coverage.launch.py'
                    )]),  launch_arguments={'use_sim_time': 'true', 'autostart': 'True', 'params_file': nav_params_file}.items()
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_headless,
        declare_world,
        declare_rviz,

        # Launch the nodes
        rviz2,
        rsp,
        joystick,
        twist_mux,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        slam,
        navigation
    ])