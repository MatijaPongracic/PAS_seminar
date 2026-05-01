#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('run_rviz')

    pkg_path = get_package_share_directory('astro')
    xacro_file = os.path.join(pkg_path, 'description', 'astro.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description],
        remappings=tf_remappings,
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('astro'), 'rviz', 'view_robot.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_config_file],
        remappings=tf_remappings,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='sim',
            description='Namespace for robot state publishers'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='false',
            description='Run RViz with astro robot config'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])

