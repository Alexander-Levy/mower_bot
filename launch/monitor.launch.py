import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction


def generate_launch_description():

    # Package name
    package_name='mower_bot' 

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='false',
        description='True uses simulation(Gazebo) clock')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')
    
    declare_slam = DeclareLaunchArgument(
        name='slam', default_value='True',
        description='Activates simultaneous localization and mapping')
    
    declare_nav = DeclareLaunchArgument(
        name='nav', default_value='True',
        description='Activates the navigation stack if true')

   
    # Launch Joystick Tele Operation
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch Twist Mux
    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': true}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    # )
        
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

    # Launch Simultaneous Localization and Mapping
    slam_node = GroupAction(
        condition=IfCondition(slam),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','slam.launch.py'
                    )]), launch_arguments={'use_sim_time': use_sim_time}.items())]
    )

    # Launch the navigation stack configured for coverage planning
    nav_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'coverage_params.yaml')
    navigation = GroupAction(
        condition=IfCondition(nav),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','coverage.launch.py'
                    )]),  launch_arguments={'use_sim_time': use_sim_time, 'autostart': 'True', 'params_file': nav_params_file}.items())]
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time,
        declare_rviz,
        declare_slam,
        declare_nav,

        # Launch the nodes
        rviz2,
        joystick,
        slam_node,
        navigation
    ])