# robot_arm_demo

Franka Panda 抓取可乐的 **ROS 2 Jazzy** 演示包（MuJoCo 仿真 + MoveIt2）。本目录是
Python 侧的全部 ROS 节点：`core/` 是机器人无关的纯逻辑，`adapters/` 是 `core` 接口的
ROS 实现，具体 demo 数值集中在 `demos/<name>/config.py`。

## 目录结构

```
robot_arm_demo/
├── core/                      # ★ 机器人无关核心逻辑（零 ROS import，pytest 可测）
│   └── data.py / interfaces.py / pick_place.py / grasp.py / command.py / camera.py
├── adapters/                  # ★ core 接口的 ROS 实现（MoveIt / gripper / 位姿源 / 感知）
│   └── moveit_arm.py / gripper_action.py / mujoco_free_joint.py / detectors.py
└── demos/panda_mujoco/        # ★ 本 demo：配置 + thin 启动节点
    └── config.py              #   全部机器人/物体/相机参数（新臂只改这里）
                              #   perception_node / pick_place_state_machine /
                              #   llm_planner / coke_pose_monitor
```

## 启动前准备（每个终端都要 source）

```bash
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
source ~/ros2_jazzy/extra_ws/install/setup.zsh
source ~/study/robot_demo/ros2_ws/install/setup.zsh
export DYLD_LIBRARY_PATH="$HOME/ros2_jazzy/extra_ws/install/mujoco_vendor/opt/mujoco_vendor/lib:${DYLD_LIBRARY_PATH}"
```

API key（预先在 `~/.zshrc` 中 `export`，或用仓库根 `.env` 覆盖）：

- `DASHSCOPE_API_KEY` —— 感知节点（Qwen-VL 兜底）
- `DEEPSEEK_API_KEY` —— LLM Planner

```bash
# 检查是否已设置（缺哪个就在 ~/.zshrc 里补哪个）
env | grep -E "DASHSCOPE_API_KEY|DEEPSEEK_API_KEY"
```

> 说明：步骤顺序是 **venv → ROS 主安装 → extra_ws → 本仓库 ros2_ws**，`setup.zsh` 会把各
> `site-packages` 加入 `PYTHONPATH`（`control_msgs`/`sensor_msgs`/`rclpy` 等）。运行 ROS
> 节点必须在这套环境里，否则会报 `No module named ...`。/ 依赖 `ros2 launch`、MoveIt 的
> C++ 插件及 `DYLD_LIBRARY_PATH` 的节点也需要 source 全部 `setup.zsh`。
>
> 开发提示：本包的 `~/ros2_jazzy/.venv` 已通过 `ros2_paths.pth` 内置 ROS 路径，PyCharm
> 用该 venv 作为 interpreter 即可自动识别 `from control_msgs...` 等导入。

## 一键启动（推荐）

```bash
cd ~/study/robot_demo
HEADLESS=false bash scripts/start-demo-mujoco.sh   # GUI 看到机械臂；true 为无 GUI
# 查看 / 关闭
tmux attach -t panda-mujoco
tmux kill-session -t panda-mujoco
```

脚本会建一个 tmux 会话（4 窗格）：**sim+MoveIt / 感知 / 状态机 / LLM**，已自动完成上述
source 与 API key 加载，无需再手动 source。

## 手动分步（调试更方便）

**终端 0：仿真 + MoveIt2**

```bash
# source 见上方「启动前准备」
ros2 launch panda_mujoco_demo panda_mujoco.launch.py headless:=false
#   headless:=true 无 GUI；false 弹 MuJoCo 窗口看机械臂
#   看到日志 "You can start planning now!" = MoveIt 就绪
```

**终端 1：感知节点**

```bash
cd ~/study/robot_demo/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.perception_node
```

**终端 2：状态机（纯物理夹持）**

```bash
cd ~/study/robot_demo/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.pick_place_state_machine
```

**终端 3：LLM Planner**

```bash
cd ~/study/robot_demo/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.llm_planner
```

> 阶段一（无需 MoveIt2）用 `panda_mujoco_sim.launch.py`；状态机依赖 `/move_action`
> (MoveIt2)，需阶段二才能完整跑。若 `ros2 launch` 找不到 `panda_mujoco_demo`，先
> `cd ~/study/robot_demo/ros2_ws && colcon build`。

## 触发

在 **LLM Planner 终端**输入：

```
帮我拿可乐
```

系统自动执行：DeepSeek 解析 → 感知定位（颜色分割 + Qwen-VL 兜底）→ MoveIt 规划 →
移动 → 两段式夹持 → 提升。

## 验证各个环节就绪

```bash
ros2 node list                    # perception_node / pick_place_state_machine / llm_planner / move_group / controller_manager ...
ros2 control list_controllers     # joint_state_broadcaster / panda_arm_controller / panda_hand_controller 均应 active
ros2 topic hz /joint_states       # 应 >0（仿真在跑）
ros2 topic echo /camera --once    # 640x480 真图
ros2 topic list | grep -E "camera|robot_command|llm_command|joint_states"
```

## 添加新机械臂 demo

`core/` 与 `adapters/` 机器人无关，新臂无需改动。只需：

1. 新建 `demos/<name>_mujoco/config.py`，按新臂填 `ArmConfig / ObjectConfig / GraspConfig /
   CameraConfig` 等数值；
2. 在 `demos/<name>_mujoco/` 下放 thin 启动节点（`perception_node.py` /
   `pick_place_state_machine.py` 等）；
3. 新增一个 `ros2_ws/src/<name>_demo` 配置包（launch + controller yaml）。

## 测试

```bash
cd ~/study/robot_demo/code/python
uv run --no-sync pytest        # core.* 的纯逻辑单测（不依赖 ROS）
```

> 若 pytest 报 `launch_testing` 插件冲突，是 ROS venv 里混装了旧版 pytest 插件所致，
> 请用 `uv` 项目 venv（Python 3.11）跑，避免与 ROS venv 混用。
