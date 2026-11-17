from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        
        DeclareLaunchArgument("target_id", default_value="0"),

        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            parameters=[{
                'marker_size': 0.1,  # Lato in metri del solo quadrato nero
                'reference_frame': 'base_link',
                'camera_frame': 'camera_optical_frame',
                'image_is_rectified': True,
                'use_sim_time': True,
            }],
            remappings=[
                ('/camera_info', '/camera/camera_info'),
                ('/image', '/camera/image_raw'),
            ],
            output='screen',
        ),

        Node(
            package='armando_controller',
            executable='armando_marker_adapter',
            name='armando_marker_adapter',
            parameters=[{
                'target_id': ParameterValue(LaunchConfiguration('target_id'), value_type=int),
                'offset_d': 0.1,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('armando_controller'),
                    'launch',
                    'armando_ik_kdl.launch.py',
                ])
            ),
        )
    ])
