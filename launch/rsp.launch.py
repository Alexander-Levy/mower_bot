import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package name
    package_name = os.path.join(get_package_share_directory('mower_bot'))
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Declare launch configs
    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true')

    declare_use_ros2_control = DeclareLaunchArgument(
            'use_ros2_control', default_value='true',
            description='Use ros2_control if true')

    # Process the URDF file and update it based on launch arguments
    xacro_file = os.path.join(package_name,'description','robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,
        node_robot_state_publisher
    ])
