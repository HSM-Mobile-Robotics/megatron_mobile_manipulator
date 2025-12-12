# MIT License
# Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab
#
# Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='micro_ros_serial_port',
            default_value='/dev/ttyACM0',
            description='Megatron teensy Serial Port'
        ),

        DeclareLaunchArgument(
            name='micro_ros_baudrate',
            default_value='460800',
            description='micro-ROS baudrate'
        ),

        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value='serial',
            description='micro-ROS transport'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration(
                "micro_ros_serial_port"), '--baudrate', LaunchConfiguration("micro_ros_baudrate")]
        )


    ])
