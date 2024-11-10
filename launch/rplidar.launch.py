from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Launch the rplidar a1m8 driver node and set-up the serial port, frame_id and scan parameters.
    rplidar = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'}]
    )

    return LaunchDescription([
        rplidar
    ])