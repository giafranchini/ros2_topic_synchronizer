from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  
    container = ComposableNodeContainer(
        name='synchronizer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='topic_synchronizer',
                plugin='synchronizer::TopicSynchronizer',
                name='topic_synchronizer',
                parameters=[os.path.join(get_package_share_directory("topic_synchronizer"), 'params', 'params.yaml')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='rosbag2_transport',
                plugin='rosbag2_transport::Player',
                name='player',
                parameters=[os.path.join(get_package_share_directory("etna_s3li_playback"), 'config', 'rosbag2_player.yaml')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=[
            '-d', PathJoinSubstitution([
                os.path.join(get_package_share_directory("etna_s3li_playback"), 'rviz', 'roxy_rviz_config.rviz')
            ]),
        ],
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('visualize', default_value='false'),
        rviz2,
        container,
    ])

