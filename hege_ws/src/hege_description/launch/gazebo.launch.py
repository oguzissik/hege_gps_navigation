import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hege_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    model = os.path.join(share, 'urdf', 'hege.urdf.xacro')
    world = os.path.join(share, 'worlds', 'flat_field.world')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world}'}.items())
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}], output='screen')
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'hege', '-allow_renaming', 'false', '-z', '0.02'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ])
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
        gazebo, state_publisher, bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[ackermann, relay])),
    ])
