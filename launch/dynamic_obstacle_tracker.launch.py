from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('dynamic_obstacle_tracker'),
        'config',
        'default.yaml',
    ])
    default_rviz_file = PathJoinSubstitution([
        FindPackageShare('dynamic_obstacle_tracker'),
        'rviz',
        'dyn.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Plain YAML file loaded by the tracker config loader',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='',
            description='Optional override for topics.dynamic_cloud_topic',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 with the packaged configuration',
        ),
        Node(
            package='dynamic_obstacle_tracker',
            executable='dynamic_obstacle_tracker_node',
            namespace='dynamic_obstacle_tracker',
            name='dynamic_obstacle_tracker_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='dynamic_obstacle_tracker_rviz',
            output='screen',
            arguments=['-d', default_rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
