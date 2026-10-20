"""
Mapping / localization stack for simulation.

Runs laser-based SLAM: AMCL + map_server (use_slam:=false) or
slam_toolbox (use_slam:=true).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_mapping(context, *args, **kwargs):
    use_slam = LaunchConfiguration("use_slam").perform(context).strip().lower()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in (
        "true",
        "1",
        "yes",
    )
    map_name = LaunchConfiguration("map_name").perform(context).strip()

    mapping_pkg = get_package_share_directory("autonombot_mapping")
    loc_pkg = get_package_share_directory("autonombot_localization")

    if use_slam in ("true", "1", "yes"):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mapping_pkg, "launch", "slam.launch.py")
                ),
                launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_pkg, "launch", "global_localization.launch.py")
            ),
            launch_arguments={
                "use_sim_time": str(use_sim_time).lower(),
                "map_name": map_name,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_slam",
            default_value="false",
            description="false=AMCL+prefab map, true=slam_toolbox online mapping",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map_name",
            default_value="small_house",
            description="use_slam:=false: prefab map under autonombot_mapping/maps/<map_name>/",
        ),
        OpaqueFunction(function=_launch_mapping),
    ])
