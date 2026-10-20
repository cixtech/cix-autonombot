#!/usr/bin/env bash
# Wait until Gazebo ign_ros2_control exposes /controller_manager, then spawn controllers.
set -uo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

CM="/controller_manager"
LIST_SVC="${CM}/list_controllers"
MAX_WAIT_SEC="${1:-120}"

echo "[wait_and_spawn_controllers] DOMAIN_ID=${ROS_DOMAIN_ID} waiting for ${LIST_SVC} (up to ${MAX_WAIT_SEC}s)..."
found=0
for ((i = 1; i <= MAX_WAIT_SEC; i++)); do
  if ros2 service list 2>/dev/null | grep -Fq "${LIST_SVC}"; then
    echo "[wait_and_spawn_controllers] ${LIST_SVC} available after ${i}s"
    found=1
    break
  fi
  sleep 1
done

if ((found == 0)); then
  echo "[wait_and_spawn_controllers] WARN: timed out waiting for ${LIST_SVC}" >&2
  echo "[wait_and_spawn_controllers] WARN: odom TF comes from odom_base_footprint_bootstrap + noisy_controller; diff_drive not spawned." >&2
  ros2 service list 2>/dev/null | head -40 || true
  exit 0
fi

exec ros2 run controller_manager spawner joint_state_broadcaster autonombot_drivers \
  --controller-manager "${CM}" \
  --controller-manager-timeout 120 \
  --service-call-timeout 30
