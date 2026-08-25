from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


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
            description='Plain YAML file shared by detector and tracker',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='',
            description='Optional override for topics.deskewed_cloud_topic',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 with the packaged configuration',
        ),
        ComposableNodeContainer(
            name='dynamic_obstacle_pipeline_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='dynamic_obstacle_tracker',
                    plugin='dynamic_obstacle_tracker::DynamicPointDetectorNode',
                    namespace='dynamic_point_detector',
                    name='dynamic_point_detector_node',
                    parameters=[{
                        'config_file': LaunchConfiguration('config_file'),
                        'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='dynamic_obstacle_tracker',
                    plugin='dynamic_obstacle_tracker::DynamicObstacleTrackerNode',
                    namespace='dynamic_obstacle_tracker',
                    name='dynamic_obstacle_tracker_node',
                    parameters=[{
                        'config_file': LaunchConfiguration('config_file'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
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
