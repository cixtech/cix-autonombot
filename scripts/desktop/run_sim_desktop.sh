#!/usr/bin/env bash
# Desktop launcher for AutonomBot Simulation.
# Wraps run_sim.sh so the terminal stays open on failure (shows the error).
set -euo pipefail

# Adjust WS_ROOT if the repo is not at ~/cix-autonombot.
WS_ROOT="${HOME}/cix-autonombot"
RUN_SIM="${WS_ROOT}/scripts/run_sim.sh"

export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"

cd "${WS_ROOT}"

if [[ ! -x "${RUN_SIM}" ]]; then
  echo "找不到启动脚本: ${RUN_SIM}"
  read -r -p "按 Enter 关闭此窗口..." _ || true
  exit 1
fi

# install/ 被删掉时无法 launch；自动走 all（先编译再启动）。
MODE="launch"
if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "[desktop] 未找到 install/setup.bash（尚未编译或已被删除）。"
  echo "[desktop] 将执行: run_sim.sh all  （清理 + 编译 + 启动）"
  MODE="all"
fi

ec=0
bash "${RUN_SIM}" "${MODE}" "$@" || ec=$?

if [[ "${ec}" -ne 0 ]]; then
  echo
  echo "启动失败（退出码 ${ec}）。"
  echo "常见原因："
  echo "  - 尚未编译 / install 被删除 → 再点一次会自动编译，或手动: ./scripts/run_sim.sh all"
  echo "  - ROS Humble 未安装 / 未 source"
  echo "  - Gazebo / 显示 (DISPLAY) 异常"
  read -r -p "按 Enter 关闭此窗口..." _ || true
fi

exit "${ec}"
