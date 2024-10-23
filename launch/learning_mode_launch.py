from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your original localization launch file
    hdl_localization_launch_path = os.path.join(
        get_package_share_directory('hdl_global_localization'),
        'launch',
        'hdl_global_localization.launch'
    )
    
    return LaunchDescription([
        # Include the localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hdl_localization_launch_path),
            launch_arguments={
                'points_topic': '/unilidar/cloud',
                'use_imu': 'true',
                'imu_topic': '/unilidar/imu',
                'odom_child_frame_id': 'unilidar_lidar'
            }.items()
        ),
        
        # Launch the recording node for pose data
        Node(
            package='learning_mode',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
            parameters=[{
                'pose_topic': '/odom',  # Make sure this matches the pose topic you're using
                'output_bag': 'pose_data'
            }]
        ),
    ])
