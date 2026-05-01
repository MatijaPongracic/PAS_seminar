from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    astro_launch = Path(get_package_share_directory("astro")) / "launch" / "rsp.launch.py"
    astro_nav_launch = (
        Path(get_package_share_directory("astro_navigation"))
        / "launch"
        / "localization_rviz.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(astro_launch))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(astro_nav_launch))
        ),
    ])
