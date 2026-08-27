# panda_mujoco_demo

Franka Panda 抓取 Demo — MuJoCo 仿真版本（由 Gazebo Harmonic 迁移而来）

---

## 目录

1. [项目概述](#1-项目概述)
2. [迁移背景：从 Gazebo 到 MuJoCo](#2-迁移背景从-gazebo-到-mujoco)
3. [架构概述](#3-架构概述)
4. [Gazebo vs MuJoCo 主要差异对比](#4-gazebo-vs-mujoco-主要差异对比)
5. [关键技术决策](#5-关键技术决策)
6. [文件目录结构](#6-文件目录结构)
7. [云服务器部署与构建](#7-云服务器部署与构建)
8. [启动 Demo](#8-启动-demo)
9. [常见问题排查](#9-常见问题排查)

---

## 1. 项目概述

用户用中文说「帮我拿可乐」，系统完成从自然语言到机械臂物理抓取的全链路：

```
用户语音/文字 → DeepSeek LLM 解析意图
             → Qwen-VL 摄像头检测可乐 2D 坐标
             → 深度反投影 + TF 坐标变换 → 3D 目标点
             → MoveIt 2 规划轨迹
             → Panda 机械臂 + 真实夹爪物理夹持可乐
```

本包 (`panda_mujoco_demo`) 负责仿真层：用 **MuJoCo + mujoco_ros2_control** 替代原 Gazebo Harmonic，其余 ROS 2 节点（感知、状态机、LLM Planner）对上层接口完全透明。

---

## 2. 迁移背景：从 Gazebo 到 MuJoCo

### 为什么迁移？

原版 Demo 基于 **Gazebo Harmonic + gz_ros2_control**，在阿里云服务器上部署存在以下问题：

- Gazebo Harmonic GUI 在无显示器的云服务器上需要完整的 OpenGL/Vulkan 驱动，headless 模式配置复杂且不稳定
- Gazebo 的 `GraspPlugin`（attach 方式）不符合真实机器人夹持需求——实机无法通过代码"焊接"物体，必须依靠物理摩擦力
- Gazebo 进程崩溃会拖垮整个仿真，重启代价高

### 为什么选 MuJoCo？

- **物理精度高**：接触力、摩擦力模拟更真实，适合验证夹持策略
- **轻量 headless**：`headless=true` 下只需 libEGL，云服务器无需 X11/GPU
- **mujoco_ros2_control**：官方 ros-controls 包，与 ROS 2 Jazzy 无缝集成，接口与 Gazebo 版完全兼容
- **DeepMind mujoco_menagerie**：提供高质量 Panda 模型，关节限位、惯性参数与实机吻合

---

## 3. 架构概述

```
┌─────────────────────────────────────────────────────┐
│  panda_mujoco.launch.py                             │
│                                                     │
│  ┌─────────────────────────────────────────────┐   │
│  │  mujoco_ros2_control (ros2_control_node)    │   │
│  │  └─ MujocoSystemInterface                  │   │
│  │      ├─ scene.xml  (MJCF 场景)             │   │
│  │      ├─ 9 关节控制接口                     │   │
│  │      └─ CameraPlugin → /camera topic       │   │
│  └────────────────┬────────────────────────────┘   │
│                   │ /joint_states, /camera          │
│  ┌────────────────▼────────────────────────────┐   │
│  │  controller_manager                         │   │
│  │  ├─ JointTrajectoryController (arm)         │   │
│  │  ├─ GripperActionController  (hand)         │   │
│  │  └─ JointStateBroadcaster                   │   │
│  └────────────────┬────────────────────────────┘   │
│                   │ FollowJointTrajectory, Gripper   │
│  ┌────────────────▼────────────────────────────┐   │
│  │  move_group (MoveIt 2)                      │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘

上层节点（独立进程）
  perception_node   ← /camera → Qwen-VL → /detection_result
  pick_place_state_machine ← /detection_result → MoveIt → Gripper
  llm_planner       ← 用户输入 → DeepSeek → /task_command
```

---

## 4. Gazebo vs MuJoCo 主要差异对比

| 方面 | Gazebo Harmonic 原版 | MuJoCo 新版 |
|------|---------------------|-------------|
| **仿真引擎** | gz-sim + gz_ros2_control | mujoco + mujoco_ros2_control |
| **机器人模型** | URDF + SDFormat | mujoco_menagerie MJCF + URDF（仅描述层） |
| **ros2_control 插件** | `GazeboSystem`（两个：arm + hand） | `MujocoSystemInterface`（单个，含全部 9 关节） |
| **夹爪控制** | `GazeboGripperPlugin` + `attach` 方式 | `GripperActionController` + 纯物理夹持 |
| **摄像头** | `gz-sensors` + `sensor_msgs` bridge | `mujoco_ros2_control_plugins/CameraPlugin` |
| **场景文件** | `.world` (SDFormat) | `scene.xml` (MJCF) |
| **Headless 支持** | 需要 Xvfb 或 gz-server 分离 | `headless=true` 参数即可，仅需 libEGL |
| **坐标系原点** | `world` 帧，`world_to_panda` 偏移 z=0.775 | `panda_link0` 即世界原点，场景向下偏移 |
| **物体抓取** | `GraspPlugin` 触发 weld/attach | 真实接触力 + 摩擦力（更接近实机） |

---

## 5. 关键技术决策

### 5.1 纯物理夹持（无 Attach）

**问题**：原版 Gazebo 通过 `gz topic -t /world/... -m gz.msgs.Entity` 将可乐"焊接"到手指上，真实机器人无法复用此逻辑。

**决策**：完全移除 attach 逻辑，改为物理夹持：
- 夹爪过盈量：可乐半径 `0.033 m`，夹持目标位置 `GRIPPER_GRASP_POS = 0.028 m`（每侧过盈 5 mm）
- 接触摩擦力：MuJoCo 默认 `friction="1 0.005 0.0001"` 足以在竖直提升时维持抓握
- `allow_stalling=true`：夹爪接触可乐后速度为零触发 stall，`reached_goal=false` 属于预期行为

```python
# pick_place_state_machine.py
GRIPPER_GRASP_POS = 0.028   # 5mm interference per side
self.send_gripper_goal(GRIPPER_GRASP_POS, max_effort=0.0)
time.sleep(1.0)             # 等待接触力稳定
```

### 5.2 坐标系重心（Re-centering）

**问题**：MJCF 无法在 `<body>` 内部使用 `<include>` 导入含 `<compiler>` 的子文件，因此无法像 Gazebo 那样在 `world` 帧下设置 `world_to_panda` 偏移。

**决策**：将 `panda_link0` 置于世界原点（`z=0`），所有场景元素向下偏移 0.775 m（原 Gazebo 的桌面高度）：

```
Gazebo 坐标系                MuJoCo 坐标系
world (z=0)                 world = panda_link0 (z=0)
  └─ panda_link0 (z=0.775)    └─ 桌面 top (z=0)
     └─ 桌面 top (z=0.775)        └─ 可乐中心 (z=0.061)
        └─ 可乐中心 (z=0.836)         └─ 摄像头 (z=0.625)
```

相机 URDF fixed joint 直接以 `panda_link0` 为父节点，无需额外的 `static_transform_publisher`。

### 5.3 完整关键帧（Full Keyframe）

**问题**：MuJoCo 的 `mj_resetDataKeyframe` 对未指定的 `qpos` 分量填零（而非 `qpos0`）。若关键帧只列出 9 个机械臂关节，可乐（freejoint，7 DOF）和桌子的位置会被清零，物体瞬移到世界原点。

**决策**：关键帧必须列出所有 `nq=16` 个分量，以及所有 `nu=8` 个 `ctrl` 分量（否则夹爪 ctrl 默认为 0，启动时立即夹紧）：

```xml
<!-- scene.xml -->
<key name="home"
  qpos="0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04
        0.3 0 0.061 1 0 0 0"
  ctrl="0 -0.785 0 -2.356 0 1.571 0.785 0.04"/>
<!--       ↑ 7 arm joints    ↑ 2 fingers
           ↑↑↑ coke freejoint: x y z qw qx qy qz
  ctrl 最后一位 0.04 = 夹爪开到 40mm（不写则启动即夹紧）-->
```

### 5.4 摄像头四元数推导

MuJoCo 摄像头沿本地 −Z 方向看，而 ROS 光学坐标系要求 +Z 朝前、+X 朝右、+Y 朝下。推导过程：

1. 确定摄像头俯视角度使可乐出现在图像中心 (320, 240)
2. 从旋转矩阵（列向量：右、下、-前）转换为四元数
3. MuJoCo 格式 `(w x y z) = (0.091956951, 0.035405950, 0.357565380, 0.928675044)`
4. URDF RPY `= (-2.4065271566321624, 0, 2.944197093726518)`
5. 验证：可乐中心 `(0.3, 0, 0.061)` 在相机坐标系下投影到 `(320, 240)`，误差 < 0.001 px

### 5.5 夹爪肌腱执行器名称约定

`mujoco_ros2_control` 对 `mjTRN_TENDON` 类型的执行器，通过执行器 **名称**（而非 joint 属性）匹配 ros2_control joint 接口。因此夹爪位置伺服必须命名为 `panda_finger_joint1`：

```xml
<!-- panda.mjcf -->
<position name="panda_finger_joint1" tendon="split"
  kp="1000" dampratio="3.0" forcerange="-20 20" ctrlrange="0 0.04"/>
```

---

## 6. 文件目录结构

```
panda_mujoco_demo/
├── CMakeLists.txt              # ament_cmake，仅安装 config/launch/mjcf/urdf
├── package.xml
├── config/
│   ├── controllers.yaml        # JointTrajectoryController + GripperActionController
│   ├── mujoco_ros2_control_plugins.yaml  # CameraPlugin 参数
│   └── initial_positions.yaml  # 机械臂初始关节角
├── launch/
│   └── panda_mujoco.launch.py  # OpaqueFunction 模式，支持 headless 参数
├── mjcf/
│   ├── panda.mjcf              # Panda 机器人（menagerie 改编，关节名对齐 URDF）
│   ├── scene.xml               # 完整场景：桌子、可乐、摄像头、完整关键帧
│   └── assets/                 # 67 个 STL/OBJ 网格文件（33 MB）
└── urdf/
    ├── panda.description.urdf.xacro   # 纯运动学描述（与 Gazebo 版完全相同）
    └── panda.mujoco.urdf.xacro        # ros2_control 插件 + camera_link 固定关节
```

### 各文件职责说明

| 文件 | 职责 |
|------|------|
| `mjcf/panda.mjcf` | 机器人关节、执行器、碰撞体定义；`<compiler>` 设置网格路径 |
| `mjcf/scene.xml` | 导入 panda.mjcf，添加环境几何体、材质、完整 home 关键帧 |
| `urdf/panda.description.urdf.xacro` | 为 robot_state_publisher 和 MoveIt 提供 URDF 链接结构 |
| `urdf/panda.mujoco.urdf.xacro` | 声明 `MujocoSystemInterface`、`CameraPlugin`、`camera_link` TF |
| `config/controllers.yaml` | ros2_control 控制器参数（与 Gazebo 版相同） |
| `config/mujoco_ros2_control_plugins.yaml` | CameraPlugin 发布参数（topic、帧率、frame_name） |
| `launch/panda_mujoco.launch.py` | 统一启动入口：仿真 + 控制器 + MoveIt |

---

## 7. 云服务器部署与构建

### 7.1 SSH 登录

```bash
ssh -i loginpair.pem allan@47.116.100.143
```

### 7.2 前置依赖确认

```bash
# 确认 ROS 2 Jazzy 已安装
source /opt/ros/jazzy/setup.bash
ros2 --version   # 应输出 ros2cli 版本

# 确认 mujoco_ros2_control 已安装或编译
ros2 pkg list | grep mujoco_ros2_control
# 如果没有输出，需要先编译 mujoco_ros2_control
```

### 7.3 构建 panda_mujoco_demo

```bash
cd ~/robot_demo_001/moveit-demo

# 仅构建本包（依赖已通过 apt 安装）
colcon build --packages-select panda_mujoco_demo --symlink-install

# 如果依赖也需要从源码构建（首次部署）：
colcon build --symlink-install
```

构建完成后 source 工作空间：

```bash
source ~/robot_demo_001/moveit-demo/install/setup.bash
```

### 7.4 配置 API Key

```bash
cp ~/robot_demo_001/.env.example ~/robot_demo_001/.env
nano ~/robot_demo_001/.env
```

`.env` 文件需填入：

```
DASHSCOPE_API_KEY=sk-...    # 阿里云 Qwen-VL API Key
DEEPSEEK_API_KEY=sk-...     # DeepSeek API Key
```

### 7.5 GUI 访问（TigerVNC）

云服务器上使用 TigerVNC 显示 MuJoCo 渲染窗口：

```bash
# 服务器端启动 VNC（若尚未运行）
vncserver :1 -geometry 1280x800 -depth 24

# 本地端口转发
ssh -i loginpair.pem -L 5901:localhost:5901 allan@47.116.100.143 -N &

# 本地 VNC 客户端连接 localhost:5901
```

若不需要图形界面（纯 headless），跳过此步骤，直接使用 `HEADLESS=true` 启动。

---

## 8. 启动 Demo

### 8.1 一键启动（推荐）

```bash
cd ~/robot_demo_001

# 有 GUI（需要 VNC）
bash scripts/start-demo-mujoco.sh

# Headless 模式（无 GUI，云服务器推荐）
HEADLESS=true bash scripts/start-demo-mujoco.sh
```

脚本会创建一个名为 `panda-mujoco` 的 tmux session，包含 4 个窗格：

| 窗格 | 内容 |
|------|------|
| 0 (左上) | MuJoCo 仿真 + MoveIt（`panda_mujoco.launch.py`） |
| 1 (右上) | 感知节点（Qwen-VL 检测可乐） |
| 2 (右下) | 状态机（物理夹持控制） |
| 3 (左下) | LLM Planner（DeepSeek 指令解析） |

### 8.2 附加到 tmux session

```bash
tmux attach -t panda-mujoco
```

在 LLM Planner 窗格（左下）输入：

```
帮我拿可乐
```

系统自动完成：检测 → 规划 → 移动 → 夹持 → 提升的全流程。

### 8.3 关闭 Demo

```bash
tmux kill-session -t panda-mujoco
```

### 8.4 手动分步启动（调试用）

如需调试单个组件，可分窗口手动启动：

```bash
# 终端 1：MuJoCo + MoveIt
source /opt/ros/jazzy/setup.bash
source ~/robot_demo_001/moveit-demo/install/setup.bash
ros2 launch panda_mujoco_demo panda_mujoco.launch.py headless:=true

# 终端 2：感知节点
source /opt/ros/jazzy/setup.bash
source ~/robot_demo_001/moveit-demo/install/setup.bash
export DASHSCOPE_API_KEY=sk-...
cd ~/robot_demo_001/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.ros2.perception_node

# 终端 3：状态机
source /opt/ros/jazzy/setup.bash
source ~/robot_demo_001/moveit-demo/install/setup.bash
cd ~/robot_demo_001/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.ros2.pick_place_state_machine

# 终端 4：LLM Planner
source /opt/ros/jazzy/setup.bash
source ~/robot_demo_001/moveit-demo/install/setup.bash
export DEEPSEEK_API_KEY=sk-...
cd ~/robot_demo_001/code/python
PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.ros2.llm_planner
```

### 8.5 启动顺序说明

1. **等待仿真就绪**：MuJoCo 加载 `scene.xml`，控制器全部激活（约 10-20 秒）
2. **等待 MoveIt**：`move_group` 就绪后输出 `Ready to take commands`
3. **感知节点**：订阅 `/camera` topic，等待图像
4. **LLM Planner**：最后启动，收到用户指令后触发全流程

---

## 9. 常见问题排查

### Q1：仿真启动后机械臂立即剧烈抖动或夹爪立即夹紧

**原因**：关键帧 `ctrl` 向量未指定，夹爪 ctrl 默认为 0（夹紧），与初始 qpos（开 40mm）产生冲突。

**检查**：确认 `scene.xml` 中关键帧有完整的 `ctrl` 属性：
```xml
ctrl="0 -0.785 0 -2.356 0 1.571 0.785 0.04"
```

### Q2：可乐或桌子在仿真开始时瞬移到原点

**原因**：关键帧 `qpos` 未列出所有自由度（freejoint 被填零）。

**检查**：`qpos` 必须有 16 个值（7 arm + 2 finger + 7 freejoint）：
```xml
qpos="0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04 0.3 0 0.061 1 0 0 0"
```

### Q3：`/camera` topic 没有图像

**排查步骤**：
```bash
ros2 topic list | grep camera
ros2 topic hz /camera
ros2 topic echo /camera/camera_info
```

**常见原因**：
- `mujoco_ros2_control_plugins.yaml` 中 `frame_name` 与 URDF 中 `camera_link` 名称不一致
- `headless=true` 时需确认 libEGL 可用：`ldconfig -p | grep EGL`

### Q4：夹爪无法夹持可乐（可乐掉落）

**检查过盈量**：`GRIPPER_GRASP_POS` 应小于可乐半径 `0.033 m`，建议值 `0.028`（5mm 过盈）。

**检查摩擦系数**：`scene.xml` 中可乐 `<geom>` 的 `friction` 属性，默认值 `"1 0.005 0.0001"` 适用于垂直提升。

### Q5：MoveIt 报告 `No motion plan found`

**排查**：
```bash
# 查看 MoveIt 规划场景
ros2 run moveit_ros_planning rviz2

# 检查 TF 完整性
ros2 run tf2_ros tf2_echo panda_link0 panda_hand
```

**常见原因**：机械臂初始姿态在碰撞区域内；检查 `config/initial_positions.yaml` 与 `scene.xml` 关键帧的 `qpos` 一致性。

### Q6：`colcon build` 报 `Package 'mujoco_ros2_control' not found`

```bash
# 方案1：从源码编译
cd ~/robot_demo_001/moveit-demo/src
git clone https://github.com/ros-controls/mujoco_ros2_control.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 方案2：apt 安装（若已有 Jazzy 二进制包）
sudo apt install ros-jazzy-mujoco-ros2-control
```

### Q7：云服务器 headless 模式下 MuJoCo 崩溃（EGL 错误）

```bash
# 安装 EGL 支持
sudo apt install libegl1 libegl-mesa0 libgl1-mesa-glx

# 设置软件渲染（无 GPU 时）
export MUJOCO_GL=osmesa
# 或
export MUJOCO_GL=egl
```

### Q8：`GripperActionController` 报 `stall detected, aborting`

这是**预期行为**：夹爪碰到可乐后停止移动即为 stall。确认 `controllers.yaml` 中设置了：
```yaml
allow_stalling: true
```
状态机应忽略此 action result 并继续执行提升动作。

---

## 附：关键参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| 可乐半径 | 0.033 m | MuJoCo cylinder size |
| 可乐高度（半） | 0.061 m | cylinder half-length |
| 可乐中心位置 | (0.3, 0, 0.061) | 相对 panda_link0 |
| 夹持目标位置 | 0.028 m | 每侧 5mm 过盈 |
| 摄像头位置 | (0.4, 0.5, 0.625) | 相对 panda_link0 |
| 摄像头视角 | 46.846° | fx = fy = 554 px |
| 摄像头分辨率 | 640×480 | |
| 控制器频率 | 100 Hz | |
| 仿真时间步 | 2ms (500Hz) | MuJoCo 默认 |
| MoveIt 规划组 | `panda_arm` | 7 DOF |
| 夹爪控制关节 | `panda_finger_joint1` | 肌腱驱动（mimic finger2） |
