import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autonombot_description = get_package_share_directory("autonombot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                autonombot_description, "urdf", "autonombot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    gazebo_cpu_affinity_arg = DeclareLaunchArgument(
        name="gazebo_cpu_affinity",
        default_value="",
        description=(
            "CPU list for `taskset -c` when starting gz sim (see gz_sim_with_taskset.launch.py). "
            "Empty string disables pinning."
        ),
    )

    world_path = PathJoinSubstitution([
            autonombot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(autonombot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("autonombot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    # Fortress (gz_version<7) uses `ign gazebo` + Ignition transport → IGN_IP.
    # Garden+ uses `gz sim` + GZ_*; set both so discovery stays on loopback.
    gz_ip_loopback = SetEnvironmentVariable(
        name="GZ_IP",
        value="127.0.0.1",
    )
    ign_ip_loopback = SetEnvironmentVariable(
        name="IGN_IP",
        value="127.0.0.1",
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("autonombot_description"),
                    "launch",
                    "gz_sim_with_taskset.launch.py",
                )]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 1 -r'"]),
                    "gz_cpu_affinity": LaunchConfiguration("gazebo_cpu_affinity"),
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "autonombot"],
    )

    # LiDAR / IMU / clock — separate from camera so large Image frames do not
    # block scan or clock in the same parameter_bridge process.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_sensor_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ],
    )

    gz_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_camera_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_cpu_affinity_arg,
        gazebo_resource_path,
        gz_ip_loopback,
        ign_ip_loopback,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_camera_bridge,
    ])
