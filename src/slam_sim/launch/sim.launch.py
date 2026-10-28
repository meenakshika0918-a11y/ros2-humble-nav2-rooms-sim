import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('slam_sim')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'diff_bot', '-topic', 'robot_description',
                   '-x', '-3.0', '-y', '1.5', '-z', '0.05', '-Y', '0.0'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rooms.world')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, rsp, spawn, rviz,
    ])
