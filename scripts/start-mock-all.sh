#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start-mock-all.sh — 一键全栈（用「云端 mock」跑新感知链路）
#
# 一个 tmux 会话里同时起五个窗格：
#   0. MuJoCo 仿真 + MoveIt2   (panda_mujoco.launch.py)
#   1. 云端位姿服务 mock       (perception_service.mock_server)
#   2. 感知节点               (cloud-first，指向 mock)
#   3. 状态机 (纯物理夹持)
#   4. LLM Planner            (需要 DEEPSEEK_API_KEY；缺失时窗格内提示缺 key)
#
# 目的：在没有 GPU 的本机，用 mock 充当云端位姿服务，验证
# 「本地感知节点 ↔ 云端契约」整条链路能跑通（sim ↔ mock ↔ 感知 ↔ 状态机）。
# 真实云端接好后，只改 POSE_SERVICE_URL，此脚本不改。
#
# 本脚本不依赖 DASHSCOPE_API_KEY（感知节点已改云端位姿链路）。
#
# 用法:
#   bash scripts/start-mock-all.sh                        # 起全栈
#   HEADLESS=true bash scripts/start-mock-all.sh          # 无 GUI sim
#   MOCK_PORT=9000 bash scripts/start-mock-all.sh         # 换 mock 端口
#   bash scripts/start-mock-all.sh --attach               # 起完进入 tmux
# 查看/关闭:
#   tmux attach -t panda-mock
#   tmux kill-session -t panda-mock
# ============================================================

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="panda-mock"
# 本机: demo 工作区(含 panda_mujoco_demo)
WS_INSTALL="$REPO_ROOT/ros2_ws/install"
# 本机: ROS2 核心 underlay
ROS_SETUP="$HOME/ros2_jazzy/install/setup.zsh"
# 本机: 3rd-party overlay (ros2_control / mujoco_vendor / mujoco_ros2_control)
EXTRA_WS="$HOME/ros2_jazzy/extra_ws/install"
# 本机: Python3.11 venv (rclpy / colcon / 运行库)
VENV="$HOME/ros2_jazzy/.venv"
PYTHON_SRC="$REPO_ROOT/code/python"

MOCK_PORT="${MOCK_PORT:-8000}"
MOCK_BASE="http://127.0.0.1:${MOCK_PORT}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()  { echo -e "${GREEN}[start-mock]${NC} $*"; }
warn() { echo -e "${YELLOW}[start-mock]${NC} $*"; }
err()  { echo -e "${RED}[start-mock]${NC} $*" >&2; }

command -v tmux >/dev/null 2>&1 || { err "tmux 未安装 (brew install tmux)"; exit 1; }
[ -f "$VENV/bin/activate" ]    || { err "找不到 venv: $VENV/bin/activate"; exit 1; }
[ -f "$ROS_SETUP" ]            || { err "找不到 ROS 2 setup: $ROS_SETUP"; exit 1; }
[ -f "$EXTRA_WS/setup.zsh" ]   || { err "找不到 extra_ws: $EXTRA_WS/setup.zsh，请先构建"; exit 1; }
[ -f "$WS_INSTALL/setup.zsh" ] || { err "找不到 install: $WS_INSTALL/setup.zsh，请先 colcon build"; exit 1; }

# API key 一律从机器环境变量读取（~/.zshrc 的 export），不依赖任何 .env 文件。
# 感知节点不再需要 DASHSCOPE_API_KEY。LLM Planner 需要 DEEPSEEK_API_KEY。
# 无论是否有 key 都保留窗格 4（缺失时 LLM 节点会在窗格内打印缺 key 错误），
# 避免「窗格被藏起来」造成困惑。
if [ -z "${DEEPSEEK_API_KEY:-}" ]; then
    warn "未检测到 DEEPSEEK_API_KEY：LLM Planner 窗格(4) 仍会启动，但输入会报缺 key。"
    warn "  如需使用，请在 ~/.zshrc 中 export DEEPSEEK_API_KEY=sk-... 后重跑。"
fi

ATTACH=false
[[ "${1:-}" == "--attach" ]] && ATTACH=true

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    warn "tmux session '$SESSION_NAME' 已存在。"
    echo "  查看:  tmux attach -t $SESSION_NAME"
    echo "  关闭:  tmux kill-session -t $SESSION_NAME"
    exit 0
fi

# ROS 需要环境（rclpy / tf2 / sensor_msgs）+ mujoco 库路径（sim/感知/状态机/LLM 用）
ros_env() {
    echo "source $VENV/bin/activate"
    echo "source $ROS_SETUP"
    echo "source $EXTRA_WS/setup.zsh"
    echo "source $WS_INSTALL/setup.zsh"
    echo "export DYLD_LIBRARY_PATH=$HOME/ros2_jazzy/extra_ws/install/mujoco_vendor/opt/mujoco_vendor/lib:\$DYLD_LIBRARY_PATH"
}

# mock 服务只需 venv（stdlib http.server，不需要 ROS）
mock_env() {
    echo "source $VENV/bin/activate"
}

HEADLESS="${HEADLESS:-false}"

log "创建 tmux session: $SESSION_NAME"
tmux new-session -d -s "$SESSION_NAME" -n "stack"

# 窗格 0: MuJoCo 仿真 + MoveIt2
log "启动 MuJoCo 仿真 + MoveIt2 (headless=${HEADLESS})..."
tmux send-keys -t "$SESSION_NAME" \
    "$(ros_env); ros2 launch panda_mujoco_demo panda_mujoco.launch.py headless:=${HEADLESS}" C-m

# 窗格 1: 云端 mock 服务
log "启动云端 mock 服务  -> $MOCK_BASE"
tmux split-window -h -t "$SESSION_NAME:0"
tmux send-keys -t "$SESSION_NAME" \
    "$(mock_env); cd $PYTHON_SRC; PYTHONPATH=src python3 -m robot_arm_demo.perception_service.mock_server $MOCK_PORT" C-m

# 窗格 2: 感知节点（cloud-first，指向 mock）
log "启动感知节点  (POSE_SERVICE_URL=$MOCK_BASE)"
tmux split-window -v -t "$SESSION_NAME:0.1"
tmux send-keys -t "$SESSION_NAME" \
    "$(ros_env); export POSE_SERVICE_URL=$MOCK_BASE; cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.perception_node" C-m

# 窗格 3: 状态机（纯物理夹持）
log "启动状态机（纯物理夹持）..."
tmux split-window -v -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME" \
    "$(ros_env); cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.pick_place_state_machine" C-m

# 窗格 4: LLM Planner（需要 DEEPSEEK_API_KEY；缺失时窗格内会打印缺 key 错误）
# 把当前 shell 的 key 值直接注入窗格，避免 tmux 未继承环境导致拿不到 key。
log "启动 LLM Planner..."
tmux split-window -v -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME" \
    "$(ros_env); export DEEPSEEK_API_KEY='${DEEPSEEK_API_KEY:-}'; cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.llm_planner" C-m

tmux select-layout -t "$SESSION_NAME" tiled

echo ""
log "一键全栈已启动！"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  tmux session:  ${GREEN}$SESSION_NAME${NC}   (0=sim 1=mock 2=感知 3=状态机 4=LLM)"
echo -e "  attach:        ${GREEN}tmux attach -t $SESSION_NAME${NC}"
echo -e "  关闭:          ${RED}tmux kill-session -t $SESSION_NAME${NC}"
echo -e "  mock 服务:     ${CYAN}$MOCK_BASE${NC}   (云端契约 docs/perception_pose_service_contract.md)"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  在 LLM Planner 窗格(4) 输入: 帮我拿可乐"
echo "  （若跳过了窗格4，可向 /llm_command 手动发一条 pick 指令触发）"
echo ""

if $ATTACH; then
    exec tmux attach -t "$SESSION_NAME"
fi
