#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('stage_ros2')
    rviz_config_file = os.path.join(pkg_path, 'config/rviz', 'crta_stage.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='sim',
            description='Namespace for RViz topics'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            output='screen',
            arguments=['-d', rviz_config_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/goal_pose', 'goal_pose'),
                ('/clicked_point', 'clicked_point'),
                ('/initialpose', 'initialpose'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ])

