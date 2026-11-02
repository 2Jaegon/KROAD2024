import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delivery_yolo',
            executable='delivery_yolo',
            name='delivery_yolo',
            output='screen',
        ),
        Node(
            package='fusion',
            executable='lidar_pre_deli',
            name='lidar_pre_deli',
            output='screen',
        ),
        Node(
            package='camera',
            executable='multi_cam',
            name='multi_cam',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d' + os.path.join(
                get_package_share_directory('fusion'),
                'rviz',
                'deli.rviz'
            )]
        )
    ])

