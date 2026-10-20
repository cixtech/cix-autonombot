#!/usr/bin/env bash
# AutonomBot simulation helper: clean stale ROS/Gazebo/RViz processes, optionally build, then launch.
#
# Usage:
#   ./scripts/run_sim.sh build              # colcon build only
#   ./scripts/run_sim.sh launch             # clean + launch (no build)
#   ./scripts/run_sim.sh all                # clean + build + launch (default)
#   ./scripts/run_sim.sh stop               # clean only (stop the running stack)
#   ./scripts/run_sim.sh launch --no-clean  # launch without killing old processes
#   ./scripts/run_sim.sh launch --no-rqt    # skip auto rqt_image_view window
#   ./scripts/run_sim.sh all -- world_name:=empty use_npu_yolo:=false
#
# Extra arguments after `--` are passed to simulated_robot.launch.py.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="${1:-all}"
shift || true

DO_CLEAN=1
OPEN_RQT=1
RQT_DELAY_SEC=20
LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-clean)
      DO_CLEAN=0
      shift
      ;;
    --no-rqt)
      OPEN_RQT=0
      shift
      ;;
    --rqt-delay)
      RQT_DELAY_SEC="${2:?--rqt-delay requires seconds}"
      shift 2
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
    *)
      # Allow legacy: first positional arg might be mode only; extra args go to launch.
      if [[ "$MODE" == "build" || "$MODE" == "launch" || "$MODE" == "all" ]]; then
        LAUNCH_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

log() { echo "[run_sim] $*"; }
warn() { echo "[run_sim] WARN: $*" >&2; }

# --- environment -------------------------------------------------------------

source_ros_setup() {
  # ROS/colcon setup.bash may reference unset vars; incompatible with `set -u`.
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

setup_env() {
  if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
    log "deactivating conda (${CONDA_DEFAULT_ENV})"
    set +u
    conda deactivate 2>/dev/null || true
    set -u
  fi

  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source_ros_setup /opt/ros/humble/setup.bash
  else
    warn "ROS Humble not found at /opt/ros/humble/setup.bash"
  fi

  # SDK packages (cix_ros_noe_yolo for NPU YOLO): provided either by the
  # installed SDK deb (visible through /opt/ros/humble) or by a robotics_sdk
  # source checkout via ROBOTICS_SDK_ROOT.
  if [[ -n "${ROBOTICS_SDK_ROOT:-}" && -f "${ROBOTICS_SDK_ROOT}/ros_packages/install/setup.bash" ]]; then
    source_ros_setup "${ROBOTICS_SDK_ROOT}/ros_packages/install/setup.bash"
  elif ! ros2 pkg prefix cix_ros_noe_yolo >/dev/null 2>&1; then
    warn "cix_ros_noe_yolo not found - NPU YOLO will be unavailable"
    warn "Install the SDK deb (cix-ros-noe-yolo), or set ROBOTICS_SDK_ROOT to a robotics_sdk checkout."
  fi

  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source_ros_setup "${WS_ROOT}/install/setup.bash"
  else
    warn "install/setup.bash missing — run: $0 build"
  fi

  export DISPLAY="${DISPLAY:-:0}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

  # CIX NPU runtime libs (libnoe.so / libaipudrv.so) for the NPU YOLO backend
  # (cix_ros_noe_yolo). Without this the node dies in EngineInfer with
  # "noe_init_context fail".
  if [[ -d /usr/share/cix/lib ]]; then
    export LD_LIBRARY_PATH="/usr/share/cix/lib:${LD_LIBRARY_PATH:-}"
  fi

  # Root for model file discovery (depth_anything_*.cix etc.).
  export ROBOTICS_DEMO_ROOT="${WS_ROOT}"
}

spawn_rqt_image_view() {
  local delay="${1:-20}"
  local view_script="${SCRIPT_DIR}/view_yolo_debug.sh"
  local log_file="/tmp/autonombot_rqt.log"

  # GUI env for the delayed child (SSH parents often lack these).
  export DISPLAY="${DISPLAY:-:0}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  if [[ -z "${XAUTHORITY:-}" ]]; then
    local xa
    xa="$(ls /run/user/"$(id -u)"/.mutter-Xwaylandauth.* 2>/dev/null | head -1 || true)"
    [[ -n "${xa}" ]] && export XAUTHORITY="${xa}"
  fi

  : >"${log_file}"
  log "scheduling rqt_image_view in ~${delay}s (topic via view_yolo_debug.sh, log: ${log_file})"
  (
    # Isolate from parent set -e so a failed rqt does not affect the sim.
    set +e
    sleep "${delay}"
    cd "${WS_ROOT}"
    echo "[$(date -Iseconds)] opening rqt_image_view via ${view_script}" >>"${log_file}"
    bash "${view_script}" >>"${log_file}" 2>&1
    echo "[$(date -Iseconds)] rqt_image_view exited: $?" >>"${log_file}"
  ) &
  disown $! 2>/dev/null || true
}

# --- cleanup -----------------------------------------------------------------

_kill_pattern() {
  local pattern="$1"
  local sig="${2:-TERM}"
  local pids
  pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    log "sending SIG${sig} to: ${pattern}"
    # shellcheck disable=SC2086
    kill -"${sig}" ${pids} 2>/dev/null || true
  fi
}

cleanup_stale_processes() {
  log "cleaning stale ROS / Gazebo / RViz processes..."

  # Prefer stopping the top-level launch first (entire subtree).
  _kill_pattern 'ros2 launch autonombot_launch' TERM
  _kill_pattern 'ros2 launch autonombot_launch' KILL

  # Gazebo Sim (Harmonic / Garden style `gz sim`).
  _kill_pattern 'gz sim' TERM
  _kill_pattern 'ruby.*gz sim' TERM
  _kill_pattern 'gz sim' KILL
  _kill_pattern 'ruby.*gz sim' KILL

  # Gazebo Sim invoked via the `ign gazebo` CLI (Gazebo Sim 6 on the board).
  # SIGTERM only for the GUI: SIGKILL'ing it soft-resets the Mali GPU and
  # hangs every later gazebo run until reboot.
  _kill_pattern 'ign gazebo' TERM
  _kill_pattern 'ruby.*ign gazebo' TERM
  _kill_pattern 'taskset.*ign gazebo' TERM

  # Legacy Gazebo Classic names (harmless if absent).
  _kill_pattern 'gzserver' TERM
  _kill_pattern 'gzclient' TERM
  _kill_pattern 'gzserver' KILL
  _kill_pattern 'gzclient' KILL

  # RViz and common AutonomBot / vision / Nav2 nodes from this stack.
  _kill_pattern 'rviz2' TERM
  _kill_pattern 'advanced_vision_navigator' TERM
  _kill_pattern 'npu_yolo_coco_inference' TERM
  _kill_pattern 'npu_yolo_' TERM
  _kill_pattern '/autonombot_' TERM
  _kill_pattern 'lifecycle_manager' TERM
  _kill_pattern 'controller_server' TERM
  _kill_pattern 'bt_navigator' TERM
  _kill_pattern 'behavior_server' TERM
  _kill_pattern 'planner_server' TERM
  _kill_pattern 'smoother_server' TERM
  _kill_pattern 'velocity_smoother' TERM
  _kill_pattern 'waypoint_follower' TERM
  _kill_pattern 'amcl' TERM
  _kill_pattern 'slam_toolbox' TERM

  sleep 1

  # Force-kill stubborn leftovers (never the gazebo GUI, see above).
  _kill_pattern 'rviz2' KILL
  _kill_pattern 'advanced_vision_navigator' KILL
  _kill_pattern 'npu_yolo_coco_inference' KILL
  _kill_pattern '/autonombot_' KILL
  _kill_pattern 'ign gazebo server' KILL
  _kill_pattern 'ruby.*ign gazebo' KILL
  _kill_pattern 'rqt_image_view' KILL

  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi

  rm -f /dev/shm/fastrtps* /dev/shm/fast_datashm* 2>/dev/null || true

  log "cleanup done"
}

# --- build -------------------------------------------------------------------

do_build() {
  log "building workspace: ${WS_ROOT}"
  cd "${WS_ROOT}"
  colcon build --symlink-install "$@"
  log "build finished — source install/setup.bash before other terminals"
}

# --- launch ------------------------------------------------------------------

default_launch_args() {
  printf '%s\n' \
    world_name:=small_house \
    use_slam:=false \
    use_advanced_vision:=true \
    use_npu_yolo:=true \
    gazebo_cpu_affinity:=0-7 \
    rviz_cpu_affinity:=9
}

do_launch() {
  setup_env

  if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    err_msg="缺少 ${WS_ROOT}/install/setup.bash — 请先编译: $0 all"
    echo "[run_sim] ERROR: ${err_msg}" >&2
    return 1
  fi
  if ! ros2 pkg prefix autonombot_launch >/dev/null 2>&1; then
    echo "[run_sim] ERROR: 找不到功能包 autonombot_launch — 请先: $0 all" >&2
    return 1
  fi

  local -a args
  mapfile -t args < <(default_launch_args)
  if [[ ${#LAUNCH_ARGS[@]} -gt 0 ]]; then
    args+=("${LAUNCH_ARGS[@]}")
  fi

  log "launching simulated_robot.launch.py with:"
  printf '  %s\n' "${args[@]}"

  if [[ "${OPEN_RQT}" -eq 1 ]]; then
    spawn_rqt_image_view "${RQT_DELAY_SEC}"
    log "rqt_image_view (/vision/yolo_debug_image) in ~${RQT_DELAY_SEC}s (use --no-rqt to disable)"
  fi

  cd "${WS_ROOT}"
  # Do not use exec: keep a chance to report errors if launch returns quickly.
  # (Desktop wrapper also pauses on non-zero exit.)
  ros2 launch autonombot_launch simulated_robot.launch.py "${args[@]}"
}

# --- main --------------------------------------------------------------------

usage() {
  cat <<EOF
Usage: $(basename "$0") [build|launch|all] [options] [-- launch-args...]

Modes:
  build    colcon build only (no cleanup, no launch)
  launch   cleanup (default) + launch simulation
  all      cleanup + build + launch (default mode)
  stop     cleanup only - stop the running simulation

Options:
  --no-clean              skip killing stale ROS/Gazebo/RViz/Gateway before launch
  --no-rqt                do not auto-open rqt_image_view after launch
  --rqt-delay N           seconds to wait before opening rqt (default: 20)

Examples:
  $(basename "$0") build
  $(basename "$0") launch
  $(basename "$0") all
  $(basename "$0") launch --no-clean
  $(basename "$0") launch --no-rqt
  $(basename "$0") all -- world_name:=small_warehouse use_npu_yolo:=false
EOF
}

case "${MODE}" in
  -h|--help|help)
    usage
    exit 0
    ;;
  build)
    setup_env
    do_build
    ;;
  stop)
    cleanup_stale_processes
    ;;
  launch)
    if [[ "${DO_CLEAN}" -eq 1 ]]; then
      cleanup_stale_processes
    fi
    do_launch
    ;;
  all)
    if [[ "${DO_CLEAN}" -eq 1 ]]; then
      cleanup_stale_processes
    fi
    setup_env
    do_build
    do_launch
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    usage >&2
    exit 1
    ;;
esac
