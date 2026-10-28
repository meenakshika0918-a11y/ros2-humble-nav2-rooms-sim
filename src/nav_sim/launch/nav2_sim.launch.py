import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nav_sim')
    slam_sim = get_package_share_directory('slam_sim')
    nav2 = get_package_share_directory('nav2_bringup')

    use_slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    # Gazebo + robot (RViz is started here with the Nav2 layout instead)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_sim, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': 'false', 'gui': gui}.items())

    nav_args = {'use_sim_time': 'true', 'params_file': params, 'autostart': 'true',
                'use_composition': 'False', 'use_respawn': 'False'}

    # Mode A (default): known map -> map_server + AMCL
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
        launch_arguments={**nav_args, 'map': map_yaml}.items(),
        condition=UnlessCondition(use_slam))

    # Mode B: slam:=true -> slam_toolbox builds the map while navigating
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_sim, 'launch', 'slam.launch.py')),
        condition=IfCondition(use_slam))

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments=nav_args.items())

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz))

    # give Gazebo time to spawn the robot before Nav2 starts
    nav_stack = TimerAction(period=12.0, actions=[localization, slam, navigation])

    # make sure AMCL gets the start pose (robot spawns at -3.0, 1.5, yaw 0)
    initial_pose = TimerAction(period=22.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '3', '-r', '1', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: map}, pose: {pose: {position: {x: -3.0, y: 1.5, z: 0.0}, '
             'orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, '
             '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}'],
        output='screen', condition=UnlessCondition(use_slam))])

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'rooms_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        sim, nav_stack, initial_pose, rviz,
    ])
