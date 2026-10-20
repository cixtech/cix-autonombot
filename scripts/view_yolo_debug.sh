#!/bin/bash
# 第二终端看图 — 与手动对齐的环境（source + DOMAIN_ID + LOCALHOST_ONLY）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

echo "正在设置ROS2 Humble环境..."

echo "加载ROS2 Humble环境..."
set +u
source /opt/ros/humble/setup.bash

echo "加载工作空间环境..."
source install/setup.bash
set -u

export ROS_DOMAIN_ID=61
echo "ROS_DOMAIN_ID设置为: ${ROS_DOMAIN_ID}"

export ROS_LOCALHOST_ONLY=1
echo "ROS_LOCALHOST_ONLY设置为: ${ROS_LOCALHOST_ONLY}"

export DISPLAY="${DISPLAY:-:0}"

echo "ROS2环境设置完成！"
echo "正在启动 rqt_image_view..."

ros2 run rqt_image_view rqt_image_view
