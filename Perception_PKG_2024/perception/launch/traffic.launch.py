import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_yolo',
            executable='traffic_yolo',
            name='traffic_yolo',
            output='screen',
        ),
        Node(
            package='camera',
            executable='yolo_traffic',
            name='yolo_traffic',
            output='screen',
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d' + os.path.join(
        #         get_package_share_directory('fusion'),
        #         'rviz',
        #         'deli.rviz'
        #     )]
        # )
    ])

