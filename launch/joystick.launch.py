import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package name
    package_name = get_package_share_directory('mower_bot')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='false',
        description='Use simulation(Gazebo) clock if true')

    # Get the params file for our joystick
    joy_params = os.path.join(package_name,'config','joystick.yaml')

    # Launch the joy Node
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    # Launches the tele-operation node and remaps its output to /cmd_vel_joy
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    # Launch!
    return LaunchDescription([
        declare_use_sim_time,
        joy_node,
        teleop_node       
    ])