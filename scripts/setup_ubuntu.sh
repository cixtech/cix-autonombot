#!/usr/bin/env bash
# AutonomBot — Ubuntu environment setup (ROS 2 Humble / Ubuntu 22.04)
#
# Usage:
#   ./scripts/setup_ubuntu.sh              # full sim stack (ROS + deps + build)
#   ./scripts/setup_ubuntu.sh --deps-only  # packages only, skip colcon build
#   ./scripts/setup_ubuntu.sh --no-vision   # skip default vision pip (opencv/numpy)
#   ./scripts/setup_ubuntu.sh --cpu-vision  # also install optional AGPL CPU YOLO stack
#   ./scripts/setup_ubuntu.sh --real       # extra real-robot apt packages (LiDAR / RealSense)
#
# After setup, each new terminal:
#   source /opt/ros/humble/setup.bash
#   source <ws>/install/setup.bash
#   # or: source <ws>/scripts/env.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DO_BUILD=1
DO_VISION=1
DO_CPU_VISION=0
DO_REAL=0
ROS_DISTRO="${ROS_DISTRO:-humble}"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[setup]${NC} $*"; }
warn() { echo -e "${YELLOW}[setup] WARN:${NC} $*" >&2; }
err()  { echo -e "${RED}[setup] ERROR:${NC} $*" >&2; }

usage() {
  sed -n '2,14p' "$0" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --deps-only)  DO_BUILD=0; shift ;;
    --no-vision)  DO_VISION=0; shift ;;
    --cpu-vision) DO_CPU_VISION=1; shift ;;
    --real)       DO_REAL=1; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) err "unknown option: $1"; usage; exit 1 ;;
  esac
done

need_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

source_ros_setup() {
  # ROS/colcon setup.bash may reference unset vars; incompatible with `set -u`.
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

# --- checks ------------------------------------------------------------------

check_os() {
  if [[ ! -f /etc/os-release ]]; then
    err "/etc/os-release missing — expected Ubuntu 22.04"
    exit 1
  fi
  # shellcheck source=/dev/null
  source /etc/os-release
  log "OS: ${PRETTY_NAME:-unknown}"
  if [[ "${ID:-}" != "ubuntu" ]]; then
    warn "This script targets Ubuntu; continuing anyway"
  fi
  if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    warn "ROS 2 Humble is officially for Ubuntu 22.04 (got ${VERSION_ID:-?})"
  fi
}

# --- apt helpers -------------------------------------------------------------

ensure_locale() {
  need_sudo apt-get update -qq
  need_sudo apt-get install -y -qq locales
  need_sudo locale-gen en_US en_US.UTF-8
  need_sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
}

install_base_tools() {
  log "Installing base build tools..."
  need_sudo apt-get install -y \
    curl wget git gnupg lsb-release software-properties-common \
    build-essential cmake \
    python3-pip python3-venv python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    libserial-dev \
    usbutils
}

install_ros_humble() {
  if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
    log "ROS 2 ${ROS_DISTRO} already present at /opt/ros/${ROS_DISTRO}"
    return 0
  fi

  log "Installing ROS 2 ${ROS_DISTRO} (desktop)..."
  need_sudo apt-get install -y curl gnupg lsb-release

  local keyring=/usr/share/keyrings/ros-archive-keyring.gpg
  if [[ ! -f "${keyring}" ]]; then
    need_sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o "${keyring}"
  fi

  local list=/etc/apt/sources.list.d/ros2.list
  if [[ ! -f "${list}" ]]; then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      | need_sudo tee "${list}" >/dev/null
  fi

  need_sudo apt-get update -qq
  need_sudo apt-get install -y "ros-${ROS_DISTRO}-desktop"
}

install_ros_packages() {
  log "Installing AutonomBot ROS apt packages..."
  # Simulation + Nav2 + common stack used by package.xml / launch files
  need_sudo apt-get install -y \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-gazebo-ros2-control" \
    "ros-${ROS_DISTRO}-ign-ros2-control" \
    "ros-${ROS_DISTRO}-ros-gz" \
    "ros-${ROS_DISTRO}-ros-gz-sim" \
    "ros-${ROS_DISTRO}-ros-gz-bridge" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-twist-mux" \
    "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
    "ros-${ROS_DISTRO}-joy" \
    "ros-${ROS_DISTRO}-joy-teleop" \
    "ros-${ROS_DISTRO}-vision-msgs" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-image-tools" \
    "ros-${ROS_DISTRO}-rqt-image-view" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    python3-transforms3d \
    python3-numpy \
    python3-opencv \
    python3-serial \
    python3-smbus \
    || warn "Some apt packages failed; rosdep may still resolve them"
}

install_real_robot_packages() {
  log "Installing real-robot sensor packages..."
  need_sudo apt-get install -y \
    "ros-${ROS_DISTRO}-rplidar-ros" \
    "ros-${ROS_DISTRO}-realsense2-camera" \
    "ros-${ROS_DISTRO}-realsense2-description" \
    || warn "Real-robot apt packages incomplete (drivers may need vendor repos)"
}

init_rosdep() {
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep..."
    need_sudo rosdep init || true
  fi
  log "Updating rosdep..."
  rosdep update
}

install_workspace_deps() {
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  log "rosdep install from ${WS_ROOT}/src ..."
  rosdep install --from-paths "${WS_ROOT}/src" --ignore-src -r -y \
    || warn "rosdep reported missing keys; continuing"
}

install_python_vision() {
  if [[ "${DO_VISION}" -ne 1 ]]; then
    log "Skipping vision pip packages (--no-vision)"
    return 0
  fi
  log "Installing default Python vision helpers (opencv/numpy; no Ultralytics)..."
  local req="${WS_ROOT}/src/autonombot_vision/requirements.txt"
  if [[ -f "${req}" ]]; then
    pip3 install --user -r "${req}"
  else
    pip3 install --user opencv-python numpy
  fi
  if [[ "${DO_CPU_VISION}" -eq 1 ]]; then
    local cpu_req="${WS_ROOT}/src/autonombot_vision/requirements-cpu-optional.txt"
    warn "Optional CPU YOLO installs ultralytics/torch/torchvision (AGPL-3.0); proceed only if you accept those terms"
    if [[ -f "${cpu_req}" ]]; then
      pip3 install --user -r "${cpu_req}"
    else
      err "Missing ${cpu_req}"
      exit 1
    fi
  else
    log "Skipping optional CPU Ultralytics stack (pass --cpu-vision to install AGPL deps)"
  fi
  warn "NPU YOLO (cix_ros_noe_yolo) needs CIX NPU on board hardware — not installed by this script"
}

ensure_env_sh() {
  local env_file="${SCRIPT_DIR}/env.sh"
  if [[ ! -f "${env_file}" ]]; then
    err "Missing ${env_file} (should ship with the repo)"
    exit 1
  fi
  chmod +x "${env_file}"
  log "Shell env helper: source ${env_file}"
}

build_workspace() {
  if [[ "${DO_BUILD}" -ne 1 ]]; then
    log "Skipping colcon build (--deps-only)"
    return 0
  fi
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
  log "Building workspace (symlink-install)..."
  cd "${WS_ROOT}"
  colcon build --symlink-install
  log "Build finished. source ${WS_ROOT}/install/setup.bash"
}

print_next_steps() {
  cat << EOF

${GREEN}========================================${NC}
 AutonomBot Ubuntu setup finished
${GREEN}========================================${NC}

  source ${SCRIPT_DIR}/env.sh
  # or:
  #   source /opt/ros/${ROS_DISTRO}/setup.bash
  #   source ${WS_ROOT}/install/setup.bash

Simulation:
  ${WS_ROOT}/scripts/run_sim.sh launch
  # or:
  #   ros2 launch autonombot_launch simulated_robot.launch.py \\
  #     world_name:=small_house use_slam:=false use_advanced_vision:=false

NPU vision is board-specific (CIX NOE) and is not installed here.
EOF
}

main() {
  log "Workspace: ${WS_ROOT}"
  check_os
  ensure_locale
  install_base_tools
  install_ros_humble
  install_ros_packages
  if [[ "${DO_REAL}" -eq 1 ]]; then
    install_real_robot_packages
  fi
  init_rosdep
  install_workspace_deps
  install_python_vision
  ensure_env_sh
  build_workspace
  print_next_steps
}

main "$@"
