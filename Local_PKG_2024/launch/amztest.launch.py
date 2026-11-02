from launch import LaunchDescription
from launch_ros.actions import Node
import os

A_gps_port = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTD4CZJD-if00-port0'
A_imu_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

B_gps_port = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTD4CZMY-if00-port0'
B_imu_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

def generate_launch_description():
    launch_description = LaunchDescription()

    # WGS84->UTM 실행
    utm_node = Node(
        package='local_pkg',
        executable='utm',
        name='utm_node',
        output='screen',
    )

    # HEADING 코드 실행
    tae_heading_node = Node(
        package='local_pkg',
        executable='tae_heading',
        name='tae_heading_node',
        output='screen',
    )

    tae_localization_node = Node(
        package='local_pkg_cpp',
        executable='tae_localization',
        name='tae_localization_node',
        output='screen',
    )

    while True:
        user_input = input("ERP 종류를 입력하세요. (A, B): ")

        if user_input.lower() == 'a' or user_input.lower() == 'A':
            print("A팀 ERP local 노드 실행")

            # GPS 코드 실행
            gps_node = Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name='nmea_serial_driver_node',
                parameters=[{'port': A_gps_port}],  # 포트 설정 추가
            )

            # IMU 코드 실행
            imu_node = Node(
                package='imu_pkg',
                executable='imu_node',
                name='imu_node',
                parameters=[{'serial_port': A_imu_port}],
            )

            launch_description.add_action(gps_node)
            # launch_description.add_action(utm_node)
            launch_description.add_action(imu_node)
            launch_description.add_action(tae_localization_node)
            break
        
        elif user_input.lower() == 'b' or user_input.lower() == 'B':
            print("B팀 ERP local 노드 실행")

            # GPS 코드 실행
            gps_node = Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name='nmea_serial_driver_node',
                parameters=[{'port': B_gps_port}],
            )

            # IMU 코드 실행
            imu_node = Node(
                package='imu_pkg',
                executable='imu_node',
                name='imu_node',
                parameters=[{'serial_port': B_imu_port}],
            )

            launch_description.add_action(gps_node)
            # launch_description.add_action(utm_node)
            launch_description.add_action(imu_node)
            launch_description.add_action(tae_localization_node)
            break
        else:
            print("잘못된 입력입니다. launch를 종료합니다.")
            exit()

    return launch_description
