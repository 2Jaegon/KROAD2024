

import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch_ros.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        get_package_share_directory('fusion'),
        'param',
        'final_gif.yaml'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to param file to load'),

        Node(
            package='fusion',
            executable='lidar_pre_amz',
            name='lidar_pre_amz',
            output='screen'),

        Node(
            package='fusion',
            executable='compare_box_image_amz',
            name='compare_box_image_amz',
            parameters=[param_dir],
            output='screen'),
        
        # Node(
        #     package='camera',
        #     executable='tri_cam_direct',
        #     name='tri_cam_direct',
        #     output='screen'),
        
        # Node(
        #     package='object_detection',
        #     executable='yolo_amz_sub',
        #     name='yolo_amz_sub',
        #     output='screen'),
        
        
        # Node(
        #     package='fusion',
        #     executable='sub_image_publisher',
        #     name='sub_image_publisher',
        #     output='screen'),

        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     namespace='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d' + os.path.join(
        #         get_package_share_directory('fusion'),
        #         'rviz',
        #         'two_bag.rviz'
        #     )]
        # )
    ])        
        
    
