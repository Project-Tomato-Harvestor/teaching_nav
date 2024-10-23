from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to your existing localization launch file
    hdl_localization_launch_path = os.path.join(
        get_package_share_directory('hdl_global_localization'),
        'launch',
        'hdl_global_localization.launch'
    )
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'bag_file',
            default_value='',
            description='Path to the ros2 bag file to replay.'
        ),
        DeclareLaunchArgument(
            'replay_topic',
            default_value='/replayed_odom',
            description='Topic to publish replayed odometry data.'
        ),

        # Launch the bag player (replayer node)
        Node(
            package='odom_bag_replay',  # Update to your replay package name
            executable='replayer_node',  # Update to your replay node executable name
            name='bag_player',
            parameters=[
                {'bag_file': LaunchConfiguration('bag_file')},
                {'replay_topic': LaunchConfiguration('replay_topic')}  # Pass the replay topic as a parameter
            ],
            output='screen'
        ),

        # Include the localization system launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hdl_localization_launch_path),
            launch_arguments={
                'points_topic': '/unilidar/cloud',
                'use_imu': 'true',
                'imu_topic': '/unilidar/imu',
                'odom_child_frame_id': 'unilidar_lidar',
            }.items()
        ),

        # Launch the simple planner
        Node(
            package='following_mode',
            executable='simple_planner',
            name='simple_planner',
            output='screen',
            parameters=[{'linear_gain': 1.0, 'angular_gain': 1.0}]
        )
    ])
