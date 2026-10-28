import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slam_sim')
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml'), {'use_sim_time': True}])
    return LaunchDescription([slam])
