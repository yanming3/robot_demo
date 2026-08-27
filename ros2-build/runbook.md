# ROS 2 Jazzy — macOS (Intel) 源码编译运行手册

> ✔️ **2026-08-25 已成功编译并验证**：`Summary: 342 packages finished`、`ros2 pkg list` = 323、talker↔listener 通信正常、rviz2 已生成。

> 依据官方文档：docs.ros.org/en/ros2_documentation/jazzy/Installation/Alternatives/macOS-Development-Setup.html
> 本机实测：macOS 14.8.9 (Sonoma) · Intel x86_64 · 12 核 / 16GB · 只剩 Command Line Tools，无完整 Xcode，无 CMake，系统 Python 3.9.6，无 colcon。

构建目录：`~/ros2_jazzy`（不进本项目 worktree，避免污染）。
自动化脚本：`build_ros2_jazzy.sh`。

---

## 第 0 步（必须手动，会重启）禁用 SIP

官方要求。SIP 会拦截进程继承 `DYLD_LIBRARY_PATH` 等动态链接环境变量。

> ⚠️ 禁用 = 降低系统安全性，属不可逆的高风险操作。成功后可在最后一步重新 `csrutil enable`。

**Intel Mac 操作：**
1. 重启 Mac，开机时按住 **⌘Command + R** 进入 Recovery 模式。
2. 顶栏菜单栏 → **实用工具 → 终端**。
3. 运行：
   ```bash
   csrutil disable
   reboot
   ```
4. 重启完成后验证：
   ```bash
   csrutil status        # 应显示 "System Integrity Protection status: disabled"
   ```

完成后继续下面的自动脚本。

---

## 运行自动构建

```bash
cd ~/study/robot_demo/ros2-build
chmod +x build_ros2_jazzy.sh
./build_ros2_jazzy.sh            # 全部 0→7 阶段
```

- 分阶段跑：`./build_ros2_jazzy.sh 0 1 2`（只跑 第0/1/2 阶段）
- 并行度调整：`JOBS=8 ./build_ros2_jazzy.sh 6`（16GB 内存建议 ≤8，避免链接时 OOM）
- 工作区目录：`WS=~/ros2_jazzy ./build_ros2_jazzy.sh`

各阶段含义：

| 阶段 | 内容 | 耗时(Intel 12核) |
|---|---|---|
| 0 | 预检(架构/Rosetta/SIP/空间/工具) | 秒 |
| 1 | `brew install` 系统依赖 | 15–45 分钟 |
| 2 | 装 `python@3.11` + 建 venv | 分钟 |
| 3 | pip 安装官方 pin 的 Python 依赖 | 分钟 |
| 4 | 写 `~/.zshrc` 环境变量(自动备份) | 秒 |
| 5 | `vcs import` 拉取全部源码 | 15–30 分钟 |
| 6 | `colcon build --symlink-install` | 1.5–4 小时 |
| 7 | 冒烟测试 talker/listener | 分钟 |

---

## 手动步骤对照（脚本等价实现）

```bash
# 1 依赖
brew install asio assimp bison bullet cmake console_bridge cppcheck \
  cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
  pyqt@5 python qt@5 sip spdlog tinyxml2

# 2+3 Python 3.11 venv + 依赖
python3.11 -m venv ~/ros2_jazzy/.venv && source ~/ros2_jazzy/.venv/bin/activate
python3 -m pip install --upgrade pip wheel
python3 -m pip install -U \
  --config-settings="--global-option=build_ext" \
  --config-settings="--global-option=-I$(brew --prefix graphviz)/include/" \
  --config-settings="--global-option=-L$(brew --prefix graphviz)/lib/" \
  argcomplete catkin_pkg colcon-common-extensions coverage cryptography \
  empy==3.3.4 flake8 flake8-blind-except==0.1.1 flake8-builtins \
  flake8-class-newline flake8-comprehensions flake8-deprecated \
  flake8-docstrings flake8-import-order flake8-quotes importlib-metadata \
  jsonschema lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces nose \
  pep8 psutil pydocstyle pydot pygraphviz pyparsing==2.4.7 pytest-mock \
  rosdep rosdistro setuptools==59.6.0 vcstool

# 4 环境变量
echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.zshrc
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)
export PATH=$PATH:$(brew --prefix qt@5)/bin

# 5 源码
mkdir -p ~/ros2_jazzy/src && cd ~/ros2_jazzy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

# 6 构建（16GB → 并行 8；新 CMake 加 policy）
cd ~/ros2_jazzy
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build --symlink-install --packages-skip-by-dep python_qt_binding \
  --executor parallel --parallel-workers 8

# 7 运行
source ~/ros2_jazzy/install/setup.zsh
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

---

## 常见问题 / 排错

| 症状 | 原因 | 处理 |
|---|---|---|
| `bootstrap.sh: line: command not found` / 工具缺失 | 未 `source venv` 或 PATH 不对 | `source ~/ros2_jazzy/.venv/bin/activate`；确认 `python3`→3.11 |
| `module 'pkgutil' has no attribute 'ImpImporter'` | 用了 Python 3.12/3.13 | **改用 Python 3.11**，重建 venv |
| 编译期 `add_library cannot create target ... already exists` | CMake 版本过新触发旧策略 | 已设 `CMAKE_POLICY_VERSION_MINIMUM=3.5` |
| 运行报 `dyld: Library not loaded ... image not found` | SIP 拦截 DYLD 或某 dylib 未链接 | 确认 `csrutil status` 为 disabled；用 `otool -L` 定位缺库 |
| `python_qt_binding` 编译失败 | Qt5/PyQt5 与 SIP 的已知问题 | 官方即跳过：`--packages-skip-by-dep python_qt_binding` |
| `rviz2` / `ogre` 编译失败 | 只有 CLT 缺完整 Xcode | 从 App Store 装完整 Xcode → `sudo xcode-select -s /Applications/Xcode.app/Contents/Developer`，重跑该包 |
| `cyclonedds` 编译失败 | 特定 DDS 在 mac 的兼容问题 | `--packages-skip cyclonedds cyclonedds`（默认 RMW 仍为 FastDDS） |
| 构建中内存不足被杀 | 并行度高 + 链接吃内存 | 降 `JOBS` 到 6/4，或 `--parallel-workers 4` |
| `ros2: command not found` | 未 source 环境 | `source ~/ros2_jazzy/install/setup.zsh` |

---

## 本机（Intel）编译必做的两处修复（实测踩坑）

**问题 1｜Ogre 报 `ld: symbol(s) not found for architecture arm64`**
`rviz_ogre_vendor/CMakeLists.txt` 在 APPLE 下**硬编码** `-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64`，Intel 机器上仍去编 arm64 切片，链 x86_64-only 的 assimp 必失败。改由宿主架构决定：
```bash
# 编辑 ~/ros2_jazzy/src/ros2/rviz/rviz_ogre_vendor/CMakeLists.txt 第81行：
#   list(APPEND OGRE_CMAKE_ARGS -DCMAKE_OSX_ARCHITECTURES=arm64;x86_64)
#   改为:
list(APPEND OGRE_CMAKE_ARGS "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_HOST_SYSTEM_PROCESSOR}")
```

**问题 2｜`python_orocos_kdl_vendor` 报 `Could not find a configuration file for package "orocos_kdl"`(请求 1.5.1 exact)**
Homebrew 现在的 `orocos-kdl` 版本(1.5.3) 比 Jazzy 的 vendor 期望(精确 1.5.1) 新，导致 `orocos_kdl_vendor` 复用了系统 1.5.3、而 PyKDL vendor 要求精确 1.5.1。修法(让 vendor 自建 1.5.1)：
```bash
brew uninstall orocos-kdl
rm -rf ~/ros2_jazzy/build/orocos_kdl_vendor ~/ros2_jazzy/build/python_orocos_kdl_vendor \
       ~/ros2_jazzy/install/orocos_kdl_vendor ~/ros2_jazzy/install/python_orocos_kdl_vendor
colcon build --symlink-install --packages-skip-by-dep python_qt_binding   # 重续
```

**但自建 KDL 还撞上 brew 新 Eigen 需要 C++14/17**（报 `Eigen/src/Core/util/Meta.h: unknown type name 'constexpr'`、`enable_if_t`）。`orocos_kdl_vendor` 虽然设了 `CMAKE_CXX_STANDARD 17`，但那是作用于 vendor 包自身，传给内层 KDL 却走 `ament_vendor` 的 `CMAKE_ARGS`，默认**未带标准参数**。需在 `~/ros2_jazzy/src/ros2/orocos_kdl_vendor/orocos_kdl_vendor/CMakeLists.txt` 的 `ament_vendor( ... CMAKE_ARGS )` 里追加：
```cmake
    -DCMAKE_CXX_STANDARD=17
    -DCMAKE_CXX_STANDARD_REQUIRED=ON
```

> 补充:colcon 并行(`--parallel-workers`) + 嵌套 make 会偶发 `make: INTERNAL: Exiting with 13 jobserver tokens available; should be 12!`,并误伤并行的兄弟包("Aborted")。多数属附带抖动,重跑一次即可;若频繁发生,把 `--parallel-workers` 调低。

**问题 3｜`python_orocos_kdl_vendor` 也被 brew 新 Eigen 的 C++14 要求卡住**
`python_orocos_kdl_vendor` 用 `add_subdirectory` 编 PyKDL,但**没设 C++ 标准** → 默认 C++11,`Eigen/.../Meta.h` 的 `common_type_t`/`make_signed_t`/`enable_if_t`/`integer_sequence`(全 C++14) 报错。在 `~/ros2_jazzy/src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/CMakeLists.txt` 的 `project()` 后、`build_pykdl()`(内部 `add_subdirectory`) 之前加上:
```cmake
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
```
改完清 `build/python_orocos_kdl_vendor`、`install/python_orocos_kdl_vendor` 再重续。

**问题 4｜报 `ModuleNotFoundError: No module named 'catkin_pkg'`(如 tf2_py)**
CMake 的 `find_package(Python3)` 会匹配到 `pyqt@5` 顺手装的 **python@3.14 框架**(无 catkin_pkg),而不是 venv 的 3.11。修法:构建前强制所有 `find_package(Python3)` 用 venv,在 colcon 前置环境即可(不会触发已建包重编):
```bash
export Python3_EXECUTABLE=~/ros2_jazzy/.venv/bin/python3
export Python_EXECUTABLE=~/ros2_jazzy/.venv/bin/python3
export PYTHON_EXECUTABLE=~/ros2_jazzy/.venv/bin/python3
```
> ⚠️ 仅 set 环境变量对**已创建的 `Python3::Interpreter` 目标**无效（`ament_package_xml` 用的是该目标）。必须用 cmake 缓存强制，判断标准是出现 `-- Found Python3: /usr/local/Frameworks/.../3.14` 且 CMakeCache 的 `_Python3_EXECUTABLE`=3.14。根治是在 colcon 加：
```bash
colcon build --symlink-install \
  --cmake-args "-DPython3_EXECUTABLE=$HOME/ros2_jazzy/.venv/bin/python3" \
             "-DPython_EXECUTABLE=$HOME/ros2_jazzy/.venv/bin/python3" \
             "-DPYTHON_EXECUTABLE=$HOME/ros2_jazzy/.venv/bin/python3" \
  --packages-skip-by-dep python_qt_binding
```

---

## 收尾（成功后再做）

确认 `ros2 run demo_nodes_cpp talker` 正常后：

- 如需恢复安全基线，**重新开启 SIP**：
  ```bash
  # 重启进恢复模式 → 终端 → csrutil enable → reboot
  csrutil enable
  ```
  注意：重开 SIP 后，若某些工具依赖 `DYLD_LIBRARY_PATH` 运行时需重新审视（多数 `--symlink-install` 走 rpath 不受影响）。
- 追加依赖用 rosdep：`rosdep update && rosdep install --from-paths ~/ros2_jazzy/src --ignore-src -r --rosdistro jazzy -y`
