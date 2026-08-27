# panda_mujoco_demo — Franka Panda 抓取可乐 (MuJoCo + ROS 2 Jazzy 本地版)

用自然语言「帮我拿可乐」驱动 Franka Panda 机械臂完成**物理抓取**的完整 demo：
DeepSeek 解析指令 → Qwen-VL + 颜色分割定位可乐 → MoveIt 2 规划 → MuJoCo 仿真里的 Panda + 夹爪物理夹持可乐。

本仓库用 **MuJoCo + mujoco_ros2_control** 作为物理仿真后端，所有依赖从源码构建，可在本机（Intel macOS）完整运行。

> 环境关键：Intel macOS、ROS 2 **Jazzy** 源码构建、**Python 3.11** venv、**SIP 已禁用**。

---

## 1. 整体架构

```
┌────────────────────────── panda_mujoco.launch.py ──────────────────────────┐
│  robot_state_publisher  — 发布 /robot_description (URDF)                    │
│  mujoco_ros2_control (ros2_control_node)                                   │
│    └─ MujocoSystemInterface  — 加载 scene.xml (MJCF), 9 关节, 物理步进      │
│        ├─ CameraPlugin            → /camera(color) + /camera/depth         │
│        └─ FreeJointStatePublisher → /free_joint_state_publisher/free_joint_states (可乐位姿)
│  controller_manager + 3 控制器:                                            │
│    joint_state_broadcaster / joint_trajectory_controller(panda_arm)        │
│    / GripperActionController(panda_hand, position_controllers)             │
│  move_group (MoveIt 2)  — 规划 + 执行 (OMPL), moveit_resources_panda       │
└────────────────────────────────────────────────────────────────────────────┘
上层节点（独立进程）
  llm_planner               ← 用户输入 → DeepSeek → /llm_command
  perception_node           ← /camera + /llm_command → Qwen-VL/颜色 → /robot_command
  pick_place_state_machine  ← /robot_command + /joint_states → MoveIt + GripperAction
```

### 数据流
```
用户: 帮我拿可乐
 → [llm_planner]  DeepSeek 解析 → /llm_command {"target_object":"可乐","action":"pick"}
 → [perception_node]  颜色分割(bounding box) → 深度反投影 + tf 相机→世界 → {action,position} → /robot_command
 → [pick_place_state_machine]  MoveIt 规划(/move_action) + 夹爪(/panda_hand_controller/gripper_cmd) + 纯物理夹持
 → [mujoco_ros2_control]  仿真执行 /joint_states 反馈
```

---

## 2. 目录结构

```
~/study/robot_demo/
├── ros2_ws/                          # 本项目工作区 (colcon, overlay 在 ~/ros2_jazzy 之上)
│   └── src/
│       ├── panda_mujoco_demo/        # ★ 本 demo 包 (纯配置/launch 包, 无 C++ 源码)
│       │   ├── config/               # controllers.yaml / mujoco_ros2_control_plugins.yaml / initial_positions.yaml
│       │   ├── launch/               # panda_mujoco.launch.py (完整, 含 move_group) / panda_mujoco_sim.launch.py (仅仿真)
│       │   ├── mjcf/                 # scene.xml (场景) + panda.mjcf + assets/ (STL/OBJ 网格)
│       │   ├── urdf/                 # panda.mujoco.urdf.xacro (ros2_control 插件+camera) / panda.description.urdf.xacro
│       │   ├── CMakeLists.txt        # ament_cmake, 仅安装 config/launch/mjcf/urdf
│       │   └── package.xml
│       └── moveit_resources/         # moveit_resources_panda_description / _moveit_config
├── code/python/                      # ROS 2 Python 节点 (uv 项目 robot_arm_demo)
│   └── src/robot_arm_demo/
│       ├── core/                      # ★ 机器人无关核心逻辑（零 ROS import，pytest 可测）
│       │   ├── data.py                #   ArmConfig/ObjectConfig/GraspConfig/CameraConfig/TaskCommand
│       │   ├── interfaces.py          #   Protocol: ArmController/Gripper/ObjectPoseSource/...
│       │   ├── pick_place.py          #   通用 pick-place 状态机 (PickPlaceController)
│       │   ├── grasp.py / command.py / camera.py   # 抓取判定 / JSON 契约 / 反投影
│       ├── adapters/                  # ★ core 接口的 ROS 实现
│       │   ├── moveit_arm.py          #   MoveGroup action + Planning Scene → ArmController
│       │   ├── gripper_action.py      #   GripperCommand action → Gripper
│       │   ├── mujoco_free_joint.py   #   FreeJointStateArray → ObjectPoseSource
│       │   └── detectors.py           #   颜色分割(主) + Qwen-VL(兜底)
│       ├── demos/panda_mujoco/        # ★ 本 demo：配置 + thin 启动节点
│       │   ├── config.py              #   全部机器人/物体/相机参数（新臂只改这里）
│       │   ├── perception_node.py     #   Qwen-VL / 颜色分割 → /robot_command
│       │   ├── pick_place_state_machine.py  # MoveIt + 夹爪 + 纯物理夹持
│       │   ├── llm_planner.py         #   DeepSeek → /llm_command
│       │   └── coke_pose_monitor.py   #   调试用 (可乐真实位姿回放)
│       └── geometry.py / trajectory.py / planar_arm.py ...  # 独立 2D 平面臂 demo
├── scripts/
│   └── start-demo-mujoco.sh          # tmux 一键启动 (4 窗格: sim+MoveIt / 感知 / 状态机 / LLM)
├── ros2-build/                       # 本机构建脚本 + runbook (ROS2/mujoco 编译 + 补丁)
└── .env.example                     # API key 模板（真实 key 经 ~/.zshrc export）
```

> **添加新机械臂 demo**：`core/` 与 `adapters/` 机器人无关，新臂无需改动它们。只需
> ① `demos/<name>_mujoco/config.py`（按新臂填 ArmConfig/ObjectConfig 等数值）+
> 薄入口节点；② `ros2_ws/src/<name>_demo` 配置包（URDF/MJCF/controllers.yaml，可仿照
> panda 包）；③ 一份 start 脚本副本。参数数值经 golden 测试锁定迁移（见 tests/）。

---

## 3. 依赖与编译（本地 macOS）

> 若你的机器和本文一致（Intel macOS + Jazzy），可照此从源码搭建。若只想运行、已有现成环境，可略过直接看第 4 节。

### 3.1 工作区与构建目录
| 目录 | 内容 | 角色 |
|---|---|---|
| `~/ros2_jazzy` | ROS 2 Jazzy **源码构建** + `.venv`(Py3.11) | 底层 underlay |
| `~/ros2_jazzy/extra_ws` | ros2_control 生态 + mujoco_vendor + mujoco_ros2_control + **MoveIt2** + 依赖 | 源码 overlay |
| `~/ros2_jazzy/extra_ws/install` | colcon 安装树 | overlay 顶层 |
| `~/study/robot_demo/ros2_ws` | panda_mujoco_demo + moveit_resources | 项目工作区 |
| `~/ros2_jazzy/{mujoco,ompl,octomap,ruckig,osqp}_stage` | 各 C++ 库**源码构建安装前缀** | 供 `find_package` |

**source 顺序**（每个要用 ROS2 的终端都要先 source）：
```bash
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
source ~/ros2_jazzy/extra_ws/install/setup.zsh
source ~/study/robot_demo/ros2_ws/install/setup.zsh
export DYLD_LIBRARY_PATH="$HOME/ros2_jazzy/extra_ws/install/mujoco_vendor/opt/mujoco_vendor/lib:${DYLD_LIBRARY_PATH}"
```
`DYLD_LIBRARY_PATH` 指向 mujoco 库目录（libmujoco rpath 缺失，SIP 已禁用所以可用）。`ros2 --version` 验证。

### 3.2 Homebrew / 系统依赖
```
brew install cmake glfw fmt eigen libomp qhull fcl nlohmann-json
# osqp 用源码装 0.6.2(见下); brew osqp(1.0.0)太新,不要用
# tmux (若用一键脚本)
```

### 3.3 C++ 库源码构建（各装到 *_stage 前缀）
```bash
# MuJoCo 3.4.0 (库 + simulate), 用 CMake 装到 ~/ros2_jazzy/mujoco_stage
# OMPL 2.x → ~/ros2_jazzy/ompl_stage  (需要 libomp: -DOpenMP_* 参数)
# OctoMap 1.9.8 → ~/ros2_jazzy/octomap_stage   (brew 1.10 无 config 且版本不符)
# Ruckig → ~/ros2_jazzy/ruckig_stage            (需用系统 nlohmann-json 3.12.0, 见 tip)
# osqp 0.6.2 → ~/ros2_jazzy/osqp_stage           (需 git submodule --init; 0.6.x 菜有 <types.h> 布局)
```
构建时都要 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`（老 CMake 兼容 CMake 4.x）。
OMPL 编译要 OpenMP：
```
-DCMAKE_CXX_FLAGS=-I$(brew --prefix libomp)/include \
-DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp" -DOpenMP_CXX_LIB_NAMES=omp \
-DOpenMP_omp_LIBRARY=$(brew --prefix libomp)/lib/libomp.dylib
```

> **关键 tip（Ruckig / nlohmann ABI）**：MoveIt 用系统 `nlohmann-json`(3.12.0)，而 ruckig 自带子模块是 3.11.3，`ruckig::CloudClient::post` 因 `json_abi` 版本不同会链接失败。修法：把系统 json.hpp 覆盖到 ruckig 的 `third_party/nlohmann/json.hpp` 再编 ruckig。

### 3.4 extra_ws 构建（ros2_control + mujoco_ros2_control + MoveIt2）
克隆的源码在 `~/ros2_jazzy/extra_ws/src/`：
- `ros-controls/ros2_control`、`ros-controls/ros2_controllers`（jazzy）、`realtime_tools`、`control_msgs`、`backward_ros`、`angles`、`filters`、`pal_statistics`(humble-devel 2.8.2, 含 REGISTER_ENTITY)、`generate_parameter_library`、`rsl`、`tl_expected`(cpp_polyfills humble 分支)、`tcb_span`/`cpp_polyfills`、`diagnostics`(ros2-jazzy)、`ros2_control_cmake`
- `moveit2`(jazzy)、`moveit_msgs`、`srdfdom`、`geometric_shapes`、`octomap_msgs`、`eigen_stl_containers`、`object_recognition_msgs`(wg-perception)、`random_numbers`、`ompl`、`mount_o`...
- `mujoco_ros2_control`（**上游 main**，含 CameraPlugin）、`mujoco_vendor`（mac 版 wrapper, 自 `moveit-demo/tools/mujoco_vendor_macos`）

构建（带 C++17 + 全部 *_stage + brew 包进 `CMAKE_PREFIX_PATH`）：
```bash
source ~/ros2_jazzy/.venv/bin/activate; source ~/ros2_jazzy/install/setup.zsh
export CMAKE_PREFIX_PATH="$HOME/ros2_jazzy/ompl_stage:$HOME/ros2_jazzy/octomap_stage:$HOME/ros2_jazzy/ruckig_stage:$HOME/ros2_jazzy/osqp_stage:$(brew --prefix qhull):$(brew --prefix fcl):$(brew --prefix eigen)"
cd ~/ros2_jazzy/extra_ws
colcon build --symlink-install --executor parallel --parallel-workers 6 \
  --cmake-args "-DBUILD_TESTING=OFF" "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
              "-DCMAKE_CXX_STANDARD=17" "-DCMAKE_CXX_STANDARD_REQUIRED=ON"
```

> Python 运行依赖（生成参数/感知等）装进 venv：`pip install jinja2 pyyaml typeguard filelock openai numpy pillow scipy xacro`

### 3.5 构建本项目
```bash
cd ~/study/robot_demo/ros2_ws
colcon build --symlink-install --packages-select panda_mujoco_demo moveit_resources_panda_moveit_config moveit_resources_panda_description
```

---

## 4. 运行 Demo

> 建议至少在 3 个独立终端（或直接跑一键脚本）。**先 source 第 3.1 节的环境**（每个终端都要）。

### 4.1 一键启动（推荐）
```bash
cd ~/study/robot_demo
HEADLESS=false bash scripts/start-demo-mujoco.sh   # GUI 看到机械臂
# 或用 tmux 查看/关闭:
tmux attach -t panda-mujoco
tmux kill-session -t panda-mujoco
```
脚本会建一个 tmux 会话（4 窗格）：sim+MoveIt / 感知 / 状态机 / LLM。

### 4.2 手动分步（调试更方便）
```bash
# 终端 0: sim + MoveIt2 (完整 launch)
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
source ~/ros2_jazzy/extra_ws/install/setup.zsh
source ~/study/robot_demo/ros2_ws/install/setup.zsh
export DYLD_LIBRARY_PATH="$HOME/ros2_jazzy/extra_ws/install/mujoco_vendor/opt/mujoco_vendor/lib:${DYLD_LIBRARY_PATH}"
# API key 已在 ~/.zshrc 中 export；若未配置可在此临时设置
ros2 launch panda_mujoco_demo panda_mujoco.launch.py headless:=false
#      ↑ headless:=true 无 GUI, false 弹 MuJoCo 窗口看机械臂
# 看到日志 "You can start planning now!" = MoveIt 就绪
```

```bash
# 终端 1: 感知节点
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh; source ~/ros2_jazzy/extra_ws/install/setup.zsh; source ~/study/robot_demo/ros2_ws/install/setup.zsh
export DYLD_LIBRARY_PATH=".../mujoco_vendor/opt/mujoco_vendor/lib:${DYLD_LIBRARY_PATH}"
# 若 ~/.zshrc 已 export 则无需此行，否则：export DASHSCOPE_API_KEY=sk-...
cd ~/study/robot_demo/code/python && PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.perception_node
```
```bash
# 终端 2: 状态机
(同上 source; 无需 API key)  PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.pick_place_state_machine
```
```bash
# 终端 3: LLM Planner
(同上 source; 若 ~/.zshrc 已 export 则无需设 key)  PYTHONPATH=src:$PYTHONPATH python3 -m robot_arm_demo.demos.panda_mujoco.llm_planner
```

### 4.3 触发
在 **LLM Planner 终端**输入：
```
帮我拿可乐
```
系统自动执行：DeepSeek 解析 → 感知定位 → MoveIt 规划 → 移动 → 夹持 → 提升。

### 4.4 验证各环节就绪
```bash
ros2 node list                      # perception_node / pick_place_state_machine / llm_planner / move_group / controller_manager ...
ros2 control list_controllers       # joint_state_broadcaster / panda_arm_controller / panda_hand_controller 都应 active
ros2 topic hz /joint_states         # 应 >0 (仿真在跑)
ros2 topic echo /camera --once      # 640x480 真图 (CGL 离屏渲染)
ros2 topic list | grep -E "camera|robot_command|llm_command|joint_states"
```

---

## 5. 配置说明

### 5.1 话题/服务
| 名称 | 类型 | 谁发 |
|---|---|---|
| `/llm_command` | std_msgs/String | llm_planner |
| `/robot_command` | std_msgs/String(JSON) | perception_node |
| `/camera`, `/camera/camera_info`, `/camera/depth` | sensor_msgs | mujoco_ros2_control `CameraPlugin` |
| `/free_joint_state_publisher/free_joint_states` | mujoco_ros2_control_msgs | `FreeJointStatePublisherPlugin` |
| `/joint_states` | sensor_msgs | joint_state_broadcaster |
| `/move_action` | moveit_msgs/MoveGroup | move_group (MoveIt) |
| `/panda_hand_controller/gripper_cmd` | control_msgs/GripperCommand | GripperActionController |

### 5.2 关键文件
- `config/mujoco_ros2_control_plugins.yaml`：camera / free_joint 插件参数。
- `config/controllers.yaml`：3 个控制器参数。
- `mjcf/scene.xml`：场景（桌子、可乐、摄像头、home 关键帧）；`<geom>` 摩擦、可乐尺寸。
- `urdf/panda.mujoco.urdf.xacro`：`MujocoSystemInterface` + `CameraPlugin` + `camera_link` TF。
- `pick_place_state_machine.py`：`GRIPPER_GRASP_POS`（夹持过盈量）、夹持高度、`max_effort`。
- **API key**：在 `~/.zshrc` 中 export `DASHSCOPE_API_KEY`(Qwen-VL) + `DEEPSEEK_API_KEY`(DeepSeek)。详见 `.env.example`。

### 5.3 关键物理解调参数（抓取）
| 参数 | 值 | 说明 |
|---|---|---|
| 可乐半径 | 0.033 m | scene.xml cylinder |
| 可乐中心 | (0.3, 0, 0.061) | 相对 panda_link0 |
| `GRIPPER_GRASP_POS` | 0.028 m | 每侧 5mm 过盈（< 半径才夹得住） |
| 可乐摩擦 | `"1 0.005 0.0001"` | 竖直提升足够抓握 |
| 相机 | 640×480, fx=fy=554 | camera_link 固定 |

---

## 6. macOS 构建与渲染说明

若在**同类型 Intel mac** 上重新编，需注意这些 macOS 特有修复（详见 `ros2-build/runbook.md`）：

### 6.1 Camera / 渲染（macOS 特有）
- **Camera 离屏渲染用 windowless CGL**（`mujoco_ros2_control/camera_plugin.cpp`），而非 GLFW——GLFW 在工作者线程建窗会触发 AppKit `NSMenu` 断言崩溃。`MUJOCO_GL` 在 3.4.0 无 Metal，只有 OpenGL。
- **MuJoCo 原生 viewer 窗口放主线程**：macOS 上 viewer 需在主线程渲染（`macos_ui.{hpp,cpp}` 用 CV 交接），并把 `ros2_control_node` 的 `executor->spin()` 挪后台线程、主线程跑 UI 任务。`headless:=false` 才能弹窗且不崩。
- `mujoco_ros2_control` 需编译 `glfw_corevideo.mm`/`macos_gui.mm`（`enable_language(OBJCXX)`）并链 `Cocoa/IOKit/CoreVideo/CoreFoundation` framework。

### 6.2 其它 mac 修复
- `ros2_control_cmake` 的 `set_compiler_options`：`-Werror=conversion/format` 降为警告（Apple Clang 的 `%lu`/sign-conversion 误报）。
- `mujoco_ros2_control_plugins` CMake：去掉 lidar 源（clang 编译失败）、`OpenGL::GL` 链接、无 EGL。
- `backward_ros`/`mujoco_ros2_control`：`--no-as-needed` 加 `if(NOT APPLE)`。
- MoveIt libc++ 专属：`collision_common.hpp` 的 `map<const pair>` 去 const；`shared_ptr::unique()`→`use_count()`。
- MoveIt `planning_pipelines` 默认只留 `["ompl"]`（chomp/pilz/stomp 未编）。

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2: command not found` | 未 source 环境 | 按 3.1 顺序 source 4 层 |
| `Library not loaded: @rpath/libmujoco.3.4.0.dylib` | libmujoco rpath 缺失 | 导出 `DYLD_LIBRARY_PATH=.../mujoco_vendor/opt/mujoco_vendor/lib` |
| 启动即崩 / `NSMenu` 断言 | GLFW 窗口在非主线程 | 用修复后的版本（camera CGL + viewer 主线程） |
| `/camera` 无图或黑帧 | CGL 上下文 / offwidth 太小 | 确认 scene.xml 设了足够 offscreen 分辨率; 看日志 "using CGL" |
| `ros2 service list`/`ros2 control` 空 | FastDDS SHM 锁冲突（`open_and_lock_file failed`） | 属警告, 通讯回退 UDP 仍可用; 换独立 `ROS_DOMAIN_ID`, 别开太多同名进程 |
| 抓取夹不住 | 夹持参数或 VLM 点误差 | 调 `GRIPPER_GRASP_POS`(更小)、摩擦、夹持高度; 降低 VLM 定位误差 |
| `move_group` 规划器崩溃 `does not exist` | 只编了 OMPL, 其它 planner 没编 | 改 launch 的 pipelines 为 `["ompl"]` |
| `colcon` 缺 `osqp/octomap/ruckig...` | 这些 C++ 库要从源码装 + 加进 `CMAKE_PREFIX_PATH` | 见 3.2/3.3 |

---

## 8. 相关技术栈
- **ROS 2 Jazzy**（rclcpp / rclpy / ros2_control / MoveIt 2 / ros2cli）
- **ros2_control**：`controller_manager`、`hardware_interface`、`joint_trajectory_controller`、`gripper_controllers`(GripperActionController)、`joint_state_broadcaster`
- **mujoco_ros2_control**：`MujocoSystemInterface`（ros2_control 系统接口适配 MuJoCo）+ `CameraPlugin` + `FreeJointStatePublisherPlugin`
- **MoveIt 2**：`move_group` + OMPL 规划器
- **MuJoCo 3.4.0**：物理引擎 + 渲染（macOS OpenGL/CGL）
- **DeepSeek**（LLM 指令解析）、**Qwen-VL (DashScope)**（视觉定位）

详见 `ros2-build/runbook.md`（完整构建排错 + 补丁）。
