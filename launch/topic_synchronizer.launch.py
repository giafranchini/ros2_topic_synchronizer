from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
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
        ],
        output='both',
    )

    return LaunchDescription([container])

