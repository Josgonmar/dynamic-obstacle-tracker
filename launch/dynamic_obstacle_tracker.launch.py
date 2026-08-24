from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('dynamic_obstacle_tracker'),
        'config',
        'default.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Plain YAML file loaded by the tracker config loader',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='dynamic_cloud',
            description='ROS topic containing the dynamic input point cloud',
        ),
        Node(
            package='dynamic_obstacle_tracker',
            executable='dynamic_obstacle_tracker_node',
            name='dynamic_obstacle_tracker_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
            }],
        ),
    ])
