from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver_node',
            namespace='/gps1',  # 네임스페이스 추가
            parameters=[{'port': '/dev/ttyACM0'}],  # 포트 설정 추가
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver_node',
            namespace='/gps2',  # 네임스페이스 추가
            parameters=[{'port': '/dev/ttyACM1'}],  # 포트 설정 추가
        ),
        Node(
                package='local_pkg',
                executable='real_heading',
                name='RealHeading_publisher_node',
                output='screen',
        ),
    ])
