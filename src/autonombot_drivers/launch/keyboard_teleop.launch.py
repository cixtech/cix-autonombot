"""Optional keyboard teleop publishing to twist_mux key_vel (separate from joy_teleop)."""

import os
import shutil

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _install_setup_bash() -> str:
    prefix = get_package_prefix("autonombot_drivers")
    return os.path.join(os.path.dirname(os.path.dirname(prefix)), "setup.bash")


def _teleop_shell_command(use_sim_time: str) -> str:
    domain = os.environ.get("ROS_DOMAIN_ID", "61")
    localhost = os.environ.get("ROS_LOCALHOST_ONLY", "1")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    setup = _install_setup_bash()
    return (
        f"export ROS_DOMAIN_ID={domain}; "
        f"export ROS_LOCALHOST_ONLY={localhost}; "
        f"export RMW_IMPLEMENTATION={rmw}; "
        "source /opt/ros/humble/setup.bash; "
        f"source {setup}; "
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
        f"--ros-args -r cmd_vel:=key_vel -p use_sim_time:={use_sim_time}; "
        "exec bash"
    )


def _terminal_execute_process(shell_cmd: str):
    if shutil.which("gnome-terminal"):
        return ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--title=Robot keyboard (i/j/k/l)",
                "--",
                "bash",
                "-lc",
                shell_cmd,
            ],
            output="screen",
            shell=False,
        )
    if shutil.which("xterm"):
        return ExecuteProcess(
            cmd=["xterm", "-e", "bash", "-lc", shell_cmd],
            output="screen",
            shell=False,
        )
    return None


def _launch_keyboard(context, *args, **kwargs):
    use_keyboard = LaunchConfiguration("use_keyboard_teleop").perform(context)
    if use_keyboard.lower() not in ("true", "1", "yes", "on"):
        return []

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    shell_cmd = _teleop_shell_command(use_sim_time)

    if not os.environ.get("DISPLAY"):
        return [
            LogInfo(
                msg=(
                    "Keyboard teleop needs DISPLAY for a terminal window. "
                    "Use second terminal (autonomous_ai_rbt/1.md)."
                )
            ),
        ]

    proc = _terminal_execute_process(shell_cmd)
    if proc is not None:
        return [
            LogInfo(
                msg=(
                    "Opening a separate terminal for keyboard teleop (i/j/k/l). "
                    "Focus that window to drive."
                )
            ),
            proc,
        ]

    return [
        LogInfo(
            msg=(
                "No terminal emulator (install gnome-terminal). "
                "Use second terminal: autonomous_ai_rbt/1.md"
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "use_keyboard_teleop",
                default_value="true",
                description="Start teleop_twist_keyboard remapped to key_vel for twist_mux",
            ),
            DeclareLaunchArgument(
                "keyboard_in_xterm",
                default_value="false",
                description="Legacy flag (ignored); gnome-terminal or xterm is used when DISPLAY is set.",
            ),
            OpaqueFunction(function=_launch_keyboard),
        ]
    )
