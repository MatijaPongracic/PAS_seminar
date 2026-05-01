import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')

    stage_ros2_dir = get_package_share_directory('stage_ros2')
    astro_dir = get_package_share_directory('astro')
    astro_navigation_dir = get_package_share_directory('astro_navigation')

    sim_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stage_ros2_dir, 'launch', 'sim_stage.launch.py')
        ),
        launch_arguments={
            'world': world,
            'namespace': namespace,
        }.items()
    )

    rsp_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astro_dir, 'launch', 'rsp_stage.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    localization_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astro_navigation_dir, 'launch', 'localization_stage.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    rviz_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stage_ros2_dir, 'launch', 'rviz_stage.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='sim',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='crta_mapa',
            description='Stage world name without .world'
        ),
        sim_stage,
        rsp_stage,
        localization_stage,
        rviz_stage,
    ])
