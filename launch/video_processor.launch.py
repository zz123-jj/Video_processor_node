import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    video_processor_dir = get_package_share_directory('video_processor')
    armor_detector_dir = get_package_share_directory('armor_detector')
    
    # Declare launch arguments
    input_video_arg = DeclareLaunchArgument(
        'input_video',
        default_value='',
        description='Path to input video file'
    )
    
    output_video_arg = DeclareLaunchArgument(
        'output_video',
        default_value='output_annotated.mp4',
        description='Path to output annotated video file'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Video playback speed multiplier'
    )
    
    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='false',
        description='Show preview window during processing'
    )
    
    # Video processor node
    video_processor_node = Node(
        package='video_processor',
        executable='video_processor_node',
        name='video_processor',
        output='screen',
        parameters=[{
            'input_video_path': LaunchConfiguration('input_video'),
            'output_video_path': LaunchConfiguration('output_video'),
            'playback_speed': LaunchConfiguration('playback_speed'),
            'show_preview': LaunchConfiguration('show_preview'),
        }]
    )
    
    # Armor detector node
    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        parameters=[
            os.path.join(armor_detector_dir, 'config', 'default.yaml')
        ]
    )
    
    return LaunchDescription([
        input_video_arg,
        output_video_arg,
        playback_speed_arg,
        show_preview_arg,
        video_processor_node,
        detector_node,
    ])
