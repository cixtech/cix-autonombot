#!/usr/bin/env python3
import os
import sys
import socket

def setup_network_isolation():
    """Localhost-only ROS graph (DOMAIN 61) with pure Fast DDS SHM (8 MB segment).

    Second terminals must source the same FASTRTPS_DEFAULT_PROFILES_FILE
    after sim launch (see scripts/env.sh).
    """

    # 1. Localhost isolation (no LAN/WAN ROS traffic)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = '61'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['IGN_IP'] = '127.0.0.1'

    # 2. Limit discovery to static localhost peer
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
    os.environ['ROS_STATIC_PEERS'] = '127.0.0.1'

    # 注意：不要设置 RMW_FASTRTPS_USE_QOS_FROM_XML=1，除非整份 XML 对所有 ROS 端点都完整；
    # 否则会与 rmw_fastrtps 的 QoS 叠加出错，常见表现是 rviz2 / 节点起不来。

    # 3. Fast DDS: pure SHM 8 MB + SYNCHRONOUS publish (matches cix_ros_noe_yolo/config/fastdds_shm_only.xml)
    fastdds_xml = '/tmp/fastdds_shm_only.xml'
    with open(fastdds_xml, 'w') as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>shm_transport</transport_id>
        <type>SHM</type>
        <segment_size>8388608</segment_size>
        <port_queue_capacity>512</port_queue_capacity>
        <healthy_check_timeout_ms>1000</healthy_check_timeout_ms>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="shm_only" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>shm_transport</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>

    <publisher profile_name="shm_publisher" is_default_profile="true">
      <qos>
        <publishMode>
          <kind>SYNCHRONOUS</kind>
        </publishMode>
      </qos>
    </publisher>
  </profiles>
</dds>""")

    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_xml

    os.environ['FASTRTPS_LOG_VERBOSITY'] = 'ERROR'

    os.system('rm -f /dev/shm/fastrtps* 2>/dev/null || true')

    print("\n" + "="*60, file=sys.stderr)
    print("🔒 Localhost ROS DDS (DOMAIN 61)", file=sys.stderr)
    print("   - ROS_LOCALHOST_ONLY=1, static peer 127.0.0.1", file=sys.stderr)
    print("   - Fast DDS: pure SHM 8 MB + SYNCHRONOUS publish", file=sys.stderr)
    print("   - Config: " + fastdds_xml, file=sys.stderr)
    print("   - Match in other terminals: FASTRTPS_DEFAULT_PROFILES_FILE + DOMAIN_ID=61", file=sys.stderr)
    print("="*60 + "\n", file=sys.stderr)

# 立即执行隔离设置
setup_network_isolation()

# ============================================
# 原有launch配置（保持不变）
# ============================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_advanced_vision = LaunchConfiguration("use_advanced_vision")
    use_finger_direction_bridge = LaunchConfiguration("use_finger_direction_bridge")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_advanced_vision_arg = DeclareLaunchArgument(
        "use_advanced_vision",
        default_value="false",
        description="Enable advanced YOLO vision navigation system with intelligent obstacle detection"
    )

    use_npu_yolo_arg = DeclareLaunchArgument(
        "use_npu_yolo",
        default_value="false",
        description=(
            "Run YOLO on CIX NPU via cix_ros_noe_yolo/npu_yolo_coco_inference.launch.py instead of CPU Ultralytics. "
            "Requires use_advanced_vision:=true (or set both true). Navigator skips local YOLO when enabled."
        ),
    )

    use_finger_direction_bridge_arg = DeclareLaunchArgument(
        "use_finger_direction_bridge",
        default_value="true",
        description=(
            "Launch autonombot_launch finger_direction_to_cmd_vel: subscribes /handpose/finger_direction, "
            "publishes key_vel. Optional SDK package cix_ros_noe_handpose in another terminal with same DOMAIN_ID; "
            "do not start a second bridge. finger_direction is provided by handpose, bridge is in autonombot_launch."
        ),
    )

    world_name = LaunchConfiguration("world_name")
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_warehouse",
        description="Gazebo world base name (no .world): small_warehouse, small_house, or empty",
    )

    gazebo_cpu_affinity_arg = DeclareLaunchArgument(
        "gazebo_cpu_affinity",
        default_value="",
        description=(
            "CPU cores for Gazebo Sim via taskset (passed to autonombot_description/gz_sim_with_taskset.launch.py). "
            "Empty disables pinning."
        ),
    )

    rviz_cpu_affinity_arg = DeclareLaunchArgument(
        "rviz_cpu_affinity",
        default_value="",
        description=(
            "CPU core(s) for RViz2 via `taskset -c` (e.g. 9, 0-3, 8-11). "
            "Default: no pinning. Use `none`/`off` or leave empty; set a core list to pin."
        ),
    )

    # Nav2 / AMCL / RViz need tf: odom -> base_footprint -> base_link from diff_drive_controller.
    # If they start before Gazebo + ros2_control are ready, local_costmap logs:
    # "Invalid frame ID odom ... frame does not exist" and RViz looks broken.
    nav_stack_delay_arg = DeclareLaunchArgument(
        "nav_stack_delay",
        default_value="10.0",
        description="Seconds to wait after sim+controller start before launching localization, Nav2, and RViz",
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": world_name,
            "gazebo_cpu_affinity": LaunchConfiguration("gazebo_cpu_affinity"),
        }.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_drivers"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_drivers"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    rviz_config = os.path.join(
        get_package_share_directory("autonombot_launch"),
        "rviz",
        "autonombot_simulation.rviz",
    )

    def delayed_nav_stack_setup(context, *args, **kwargs):
        affinity = LaunchConfiguration("rviz_cpu_affinity").perform(context).strip()
        if not affinity or affinity.lower() in ("none", "off", "false", "no"):
            rviz_prefix = None
        else:
            rviz_prefix = f"taskset -c {affinity}"
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            prefix=rviz_prefix,
            arguments=["-d", rviz_config],
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
        return [
            TimerAction(
                period=LaunchConfiguration("nav_stack_delay"),
                actions=[localization, slam, navigation, rviz],
            )
        ]

    # /handpose/finger_direction (optional cix_ros_noe_handpose elsewhere) -> key_vel; bridge is in this package
    handpose_finger_bridge = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("autonombot_launch"),
            "launch",
            "finger_direction_to_cmd_vel.launch.py",
        ),
        condition=IfCondition(use_finger_direction_bridge),
    )

    advanced_vision_cpu = Node(
        package="autonombot_vision",
        executable="advanced_vision_navigator",
        name="advanced_vision_navigator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "conf_thr": 0.25,
                "nms_thr": 0.45,
                "use_cpu_yolo": True,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_advanced_vision,
                    "' == 'true' and '",
                    LaunchConfiguration("use_npu_yolo"),
                    "' != 'true'",
                ]
            )
        ),
    )

    advanced_vision_npu = Node(
        package="autonombot_vision",
        executable="advanced_vision_navigator",
        name="advanced_vision_navigator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "conf_thr": 0.25,
                "nms_thr": 0.45,
                "use_cpu_yolo": False,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_advanced_vision,
                    "' == 'true' and '",
                    LaunchConfiguration("use_npu_yolo"),
                    "' == 'true'",
                ]
            )
        ),
    )

    # cix_ros_noe_yolo ships in the CIX robotics SDK (deb or ROBOTICS_SDK_ROOT
    # checkout); degrade gracefully when only the CPU path is available.
    try:
        _npu_yolo_launch = os.path.join(
            get_package_share_directory("cix_ros_noe_yolo"),
            "launch",
            "npu_yolo_coco_inference.launch.py",
        )
    except PackageNotFoundError:
        _npu_yolo_launch = None

    if _npu_yolo_launch is not None:
        npu_yolo = IncludeLaunchDescription(
            _npu_yolo_launch,
            launch_arguments={
                "use_sim_time": "true",
                "conf_thr": "0.25",
                "nms_thr": "0.45",
                "image_topic": "/camera/image_raw",
                "detections_topic": "/vision/detections_2d",
                "npu_active_topic": "/npu_vision_active",
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        use_advanced_vision,
                        "' == 'true' and '",
                        LaunchConfiguration("use_npu_yolo"),
                        "' == 'true'",
                    ]
                )
            ),
        )
    else:
        npu_yolo = LogInfo(
            msg="[autonombot] cix_ros_noe_yolo not found - NPU YOLO unavailable "
                "(install the CIX robotics SDK deb, or source a ROBOTICS_SDK_ROOT checkout)"
        )

    return LaunchDescription([
        use_slam_arg,
        use_advanced_vision_arg,
        use_npu_yolo_arg,
        use_finger_direction_bridge_arg,
        world_name_arg,
        gazebo_cpu_affinity_arg,
        rviz_cpu_affinity_arg,
        nav_stack_delay_arg,
        gazebo,
        controller,
        joystick,
        handpose_finger_bridge,
        OpaqueFunction(function=delayed_nav_stack_setup),
        advanced_vision_cpu,
        advanced_vision_npu,
        npu_yolo,
    ])