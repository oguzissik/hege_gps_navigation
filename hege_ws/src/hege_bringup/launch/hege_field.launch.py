"""HEGE field/PX4 SITL: retain the real stack's mux, bridge and sensor adapter."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('hege_bringup')
    sensors = PathJoinSubstitution([FindPackageShare('hege_px4_sensors'), 'config', 'sensors.yaml'])
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '74'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8889'], output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[PathJoinSubstitution([bringup, 'config', 'twist_mux.yaml'])],
             remappings=[('cmd_vel_out', '/cmd_vel/selected')], output='screen'),
        Node(package='hege_px4_bridge', executable='bridge', name='hege_px4_bridge',
             parameters=[PathJoinSubstitution([bringup, 'config', 'bridge_hege_field.yaml'])], output='screen'),
        Node(package='hege_px4_sensors', executable='sensors', name='hege_px4_sensors',
             parameters=[sensors], output='screen'),
    ])
