from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('h2017_with_delto_moveit')
    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[bridge_yaml],
        output='screen'
    )

    return LaunchDescription([bridge_node])