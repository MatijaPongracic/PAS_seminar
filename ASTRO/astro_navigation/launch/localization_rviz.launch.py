import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('run_rviz')
    map_file = LaunchConfiguration('map')

    astro_dir = get_package_share_directory('astro')
    astro_navigation_dir = get_package_share_directory('astro_navigation')

    rviz_config = os.path.join(
        astro_navigation_dir, 'config', 'rviz', 'crta_stage.rviz'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astro_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'run_rviz': 'false',
            'run_jspg': 'false',
            'run_ekf': 'true',
        }.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astro_navigation_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                astro_navigation_dir, 'map', 'crta_mapa.yaml'
            )
        ),
        rsp,
        localization,
        rviz,
    ])
