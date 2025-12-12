# MIT License
# Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab
#
# Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    id = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('megatron_tools'),
        'config',
        'step_cmd_params.yaml'
    ),
    mpc_step_cmd_publisher_node = Node(
        package='megatron_tools',
        executable='step_cmd_publisher',
        name='step_cmd_publisher_node',
        parameters=[config]
    )

    id.add_action(mpc_step_cmd_publisher_node)

    return id
