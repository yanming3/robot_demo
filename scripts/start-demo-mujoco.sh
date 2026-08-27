#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start-demo-mujoco.sh — Panda pick-place demo on MuJoCo 一键启动（本机 macOS 版）
#
# 窗格:
#   0. MuJoCo 仿真 (+ MoveIt2, 需阶段二)  (panda_mujoco.launch.py)
#   1. Perception node    (VLM 感知)
#   2. State machine      (pure physical gripper — no attach)
#   3. LLM Planner        (自然语言 → JSON 指令)
#
# 本机改动（vs 云主机版）:
#   * 路径: ROS_SETUP / extra_ws / ros2_ws 均指向本机; 额外 source ~/ros2_jazzy/.venv
#   * 阶段一(无 MoveIt2): 窗格 0 请改用 panda_mujoco_sim.launch.py（见下方注释），
#     STATE MACHINE 窗格依赖 /move_action(MoveIt2)，需阶段二才能完整跑。
# ============================================================

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="panda-mujoco"
# 本机: demo 工作区(含 panda_mujoco_demo)。阶段二加 MoveIt2 后仍在此 overlay。
WS_INSTALL="$REPO_ROOT/ros2_ws/install"
# 本机: ROS2 核心 underlay
ROS_SETUP="$HOME/ros2_jazzy/install/setup.zsh"
# 本机: 3rd-party overlay (ros2_control / mujoco_vendor / mujoco_ros2_control)
EXTRA_WS="$HOME/ros2_jazzy/extra_ws/install"
# 本机: Python3.11 venv (rclpy / colcon / 运行库)
VENV="$HOME/ros2_jazzy/.venv"
PYTHON_SRC="$REPO_ROOT/code/python"
ENV_FILE="$REPO_ROOT/.env"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()  { echo -e "${GREEN}[start-mujoco]${NC} $*"; }
warn() { echo -e "${YELLOW}[start-mujoco]${NC} $*"; }
err()  { echo -e "${RED}[start-mujoco]${NC} $*" >&2; }

command -v tmux >/dev/null 2>&1 || { err "tmux 未安装 (brew install tmux)"; exit 1; }
[ -f "$ROS_SETUP" ]                   || { err "找不到 ROS 2 setup: $ROS_SETUP"; exit 1; }
[ -f "$EXTRA_WS/setup.zsh" ]          || { err "找不到 extra_ws: $EXTRA_WS/setup.zsh，请先构建"; exit 1; }
[ -f "$WS_INSTALL/setup.zsh" ]        || { err "找不到 install: $WS_INSTALL/setup.zsh，请先 colcon build"; exit 1; }
# API key 来源：~/.zshrc 的 export（推荐）。可选 source .env 作为本地覆盖。
if [ -f "$ENV_FILE" ]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    set +a
fi

# 校验 API key（来源：环境变量 或 .env）
[ -n "${DASHSCOPE_API_KEY:-}" ] || { err "缺少 DASHSCOPE_API_KEY：请在 ~/.zshrc 中 export，或写入 $ENV_FILE"; exit 1; }
[ -n "${DEEPSEEK_API_KEY:-}" ]  || { err "缺少 DEEPSEEK_API_KEY：请在 ~/.zshrc 中 export，或写入 $ENV_FILE"; exit 1; }

ATTACH=false
[[ "${1:-}" == "--attach" ]] && ATTACH=true

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    warn "tmux session '$SESSION_NAME' 已存在。"
    echo "  查看:  tmux attach -t $SESSION_NAME"
    echo "  关闭:  tmux kill-session -t $SESSION_NAME"
    exit 0
fi

common_env() {
    echo "source $VENV/bin/activate"
    echo "source $ROS_SETUP"
    echo "source $EXTRA_WS/setup.zsh"
    echo "source $WS_INSTALL/setup.zsh"
    echo "export DYLD_LIBRARY_PATH=$HOME/ros2_jazzy/extra_ws/install/mujoco_vendor/opt/mujoco_vendor/lib:\$DYLD_LIBRARY_PATH"
}

HEADLESS="${HEADLESS:-false}"

log "创建 tmux session: $SESSION_NAME"
tmux new-session -d -s "$SESSION_NAME" -n "mujoco"
# 阶段二已有 MoveIt2: 用完整 panda_mujoco.launch.py(含 move_group)
LAUNCH_FILE="panda_mujoco.launch.py"
tmux send-keys -t "$SESSION_NAME" \
    "$(common_env); ros2 launch panda_mujoco_demo $LAUNCH_FILE headless:=${HEADLESS}" C-m

log "启动感知节点..."
tmux split-window -h -t "$SESSION_NAME:0"
tmux send-keys -t "$SESSION_NAME" \
    "$(common_env); export DASHSCOPE_API_KEY; cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.panda_mujoco.perception_node" C-m

log "启动状态机（纯物理夹持）..."
tmux split-window -v -t "$SESSION_NAME:0.1"
tmux send-keys -t "$SESSION_NAME" \
    "$(common_env); cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.panda_mujoco.pick_place_state_machine" C-m

log "启动 LLM Planner..."
tmux split-window -v -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME" \
    "$(common_env); export DEEPSEEK_API_KEY; cd $PYTHON_SRC; PYTHONPATH=src:\$PYTHONPATH python3 -m robot_arm_demo.panda_mujoco.llm_planner" C-m

tmux select-layout -t "$SESSION_NAME" tiled

echo ""
log "MuJoCo demo 启动完成！"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  tmux session:  ${GREEN}$SESSION_NAME${NC}"
echo -e "  attach:        ${GREEN}tmux attach -t $SESSION_NAME${NC}"
echo -e "  关闭:          ${RED}tmux kill-session -t $SESSION_NAME${NC}"
echo -e "  headless 模式: ${YELLOW}HEADLESS=true bash scripts/start-demo-mujoco.sh${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  在 LLM Planner 窗格 (3) 输入: 帮我拿可乐"
echo "  (阶段一无 MoveIt2，状态机窗格可能报 /move_action 缺失，属预期)"
echo ""

if $ATTACH; then
    exec tmux attach -t "$SESSION_NAME"
fi
