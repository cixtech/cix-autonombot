#!/usr/bin/env bash
# Restart RViz for AutonomBot simulation (CPU affinity 9), with DDS/env aligned to sim.
#
# Usage:
#   ./scripts/run_rviz.sh
#   AUTONOMBOT_WS=~/cix-autonombot ./scripts/run_rviz.sh
#
# Requires simulation already running (writes /tmp/fastdds_shm_only.xml).

set -euo pipefail

WS_ROOT="${AUTONOMBOT_WS:-${HOME}/cix-autonombot}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${WS_ROOT}/install/autonombot_launch/share/autonombot_launch/rviz/autonombot_simulation.rviz}"
RVIZ_CPU="${RVIZ_CPU:-9}"

log() { echo "[run_rviz] $*"; }
warn() { echo "[run_rviz] WARN: $*" >&2; }

# Avoid conda shadowing ROS Python
if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
  set +u
  conda deactivate 2>/dev/null || true
  set -u
fi

export DISPLAY="${DISPLAY:-:0}"

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
else
  set -u
  echo "[run_rviz] ERROR: 缺少 ${WS_ROOT}/install/setup.bash — 请先编译该工作区" >&2
  read -r -p "按 Enter 关闭此窗口..." _ || true
  exit 1
fi
set -u

# Align with simulated_robot.launch.py setup_network_isolation()
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
export ROS_STATIC_PEERS=127.0.0.1
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_shm_only.xml

if [[ ! -f /tmp/fastdds_shm_only.xml ]]; then
  warn "/tmp/fastdds_shm_only.xml 不存在 — 请先启动仿真，否则可能看不到 TF/话题"
fi

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "[run_rviz] ERROR: 找不到 RViz 配置: ${RVIZ_CONFIG}" >&2
  read -r -p "按 Enter 关闭此窗口..." _ || true
  exit 1
fi

log "workspace: ${WS_ROOT}"
log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-?}"
log "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
log "killing existing rviz2 (if any)..."
pkill -f '[r]viz2' 2>/dev/null || true
sleep 1
pkill -9 -f '[r]viz2' 2>/dev/null || true

log "starting: taskset -c ${RVIZ_CPU} rviz2 -d ${RVIZ_CONFIG} --ros-args -p use_sim_time:=true"
ec=0
taskset -c "${RVIZ_CPU}" rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true || ec=$?

if [[ "${ec}" -ne 0 ]]; then
  echo
  echo "RViz 退出码 ${ec}。"
  echo "请确认仿真已启动，且 DOMAIN/DDS 与主栈一致。"
  read -r -p "按 Enter 关闭此窗口..." _ || true
fi

exit "${ec}"
