#!/usr/bin/env bash
# AutonomBot shell environment — source this in every terminal that talks to the stack.
#
#   source scripts/env.sh
#
# Must match simulated_robot.launch.py setup_network_isolation()
# (ROS_DOMAIN_ID=61, localhost-only, Fast DDS SHM profile).

_ab_nounset=0
case $- in *u*) _ab_nounset=1 ;; esac
set +u

# Avoid conda shadowing system Python/ROS
if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
  conda deactivate 2>/dev/null || true
fi

# Match the sim exactly. These are set unconditionally: a pre-existing value
# (e.g. ROS_LOCALHOST_ONLY=0 from a shell profile) would otherwise hide the
# SHM-only sim from this terminal.
export ROS_DOMAIN_ID=61
export ROS_LOCALHOST_ONLY=1
export DISPLAY="${DISPLAY:-:0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "[env.sh] ERROR: /opt/ros/humble/setup.bash not found — run scripts/setup_ubuntu.sh" >&2
  [[ "${_ab_nounset}" -eq 1 ]] && set -u
  unset _ab_nounset
  return 1 2>/dev/null || exit 1
fi

_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOTICS_DEMO_ROOT="${_WS_ROOT}"

# SDK packages (cix_ros_noe_yolo for NPU YOLO): provided either by the
# installed SDK deb (visible through /opt/ros/humble) or by a robotics_sdk
# source checkout via ROBOTICS_SDK_ROOT.
if [[ -n "${ROBOTICS_SDK_ROOT:-}" && -f "${ROBOTICS_SDK_ROOT}/ros_packages/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROBOTICS_SDK_ROOT}/ros_packages/install/setup.bash"
elif ! ros2 pkg prefix cix_ros_noe_yolo >/dev/null 2>&1; then
  echo "[env.sh] WARN: cix_ros_noe_yolo not found - NPU YOLO will be unavailable" >&2
  echo "[env.sh]        Install the SDK deb (cix-ros-noe-yolo), or set ROBOTICS_SDK_ROOT to a robotics_sdk checkout." >&2
fi

if [[ -f "${_WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${_WS_ROOT}/install/setup.bash"
else
  echo "[env.sh] WARN: ${_WS_ROOT}/install/setup.bash missing — run: colcon build" >&2
fi

# Fast DDS SHM profile: after simulated_robot.launch.py starts it writes
# /tmp/fastdds_shm_only.xml; a terminal without this env sees no topics from
# the (shared-memory-only) sim.
if [[ -f /tmp/fastdds_shm_only.xml ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_shm_only.xml
fi

[[ "${_ab_nounset}" -eq 1 ]] && set -u
unset _ab_nounset _WS_ROOT
