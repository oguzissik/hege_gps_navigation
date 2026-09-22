import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    share = get_package_share_directory('hege_description')
    model = os.path.join(share, 'urdf', 'hege.urdf.xacro')
    rviz = os.path.join(share, 'rviz', 'hege.rviz')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': False}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz], output='screen'),
    ])
