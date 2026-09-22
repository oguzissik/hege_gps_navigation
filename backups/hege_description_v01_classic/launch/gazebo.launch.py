import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hege_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    model = os.path.join(share, 'urdf', 'hege.urdf.xacro')
    world = os.path.join(share, 'worlds', 'flat_field.world')
    xml = xacro.process_file(model).documentElement.toxml()
    xml = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)
    robot_description = ParameterValue(xml, value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}], output='screen')
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'hege', '-z', '0.02'])
    joint_states = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen')
    ackermann = Node(
        package='controller_manager', executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'], output='screen')
    relay = Node(
        package='hege_description', executable='cmd_vel_relay.py',
        parameters=[{'use_sim_time': True}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gzserver, gzclient, state_publisher, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[ackermann, relay])),
    ])
