#!/usr/bin/env bash
#
# ROS 2 Jazzy — macOS (Intel, x86_64) source build 自动化脚本
# ============================================================
# 依据官方文档:
#   docs.ros.org/en/ros2_documentation/jazzy/Installation/Alternatives/macOS-Development-Setup.html
#
# 前提（请先手动完成，见 runbook.md 第 0 步）:
#   1. SIP 已禁用（csrutil status 应显示 disabled）
#   2. Command Line Tools 已装（$ xcode-select -p 有输出）
#   3. Homebrew 已装
#
# 本脚本做前半段「确定性」步骤; 构造/冒烟在第 6~7 段, 按需运行。
# 用法:
#   chmod +x build_ros2_jazzy.sh
#   ./build_ros2_jazzy.sh            # 默认跑 1->7 全部
#   ./build_ros2_jazzy.sh 1 2 3      # 只跑指定阶段
#
# 环境可调:
#   WS=~/ros2_jazzy            工作区目录
#   JOBS=8                    并行度(16GB 内存, 别设满 12)
#   PY=python@3.11            编译用 Python(Must be 3.11)
#
set -euo pipefail

WS="${WS:-$HOME/ros2_jazzy}"
JOBS="${JOBS:-8}"
PY=python@3.11
LOG_DIR="$WS/.build-logs"
mkdir -p "$LOG_DIR"

log()  { printf '\n\033[1;34m>>> %s\033[0m\n' "$*"; }
err()  { printf '\033[1;31m!! ERROR: %s\033[0m\n' "$*"; }

need() { if ! command -v "$1" >/dev/null 2>&1; then err "缺少命令: $1"; exit 1; fi; }

# ---------------------------------------------------------------------------
phase0_preflight() {
  log "Phase 0: 预检"
  echo "架构: $(uname -m)"
  if [ "$(uname -m)" != "x86_64" ]; then
    echo "注意: 非 x86_64。若在 Apple Silicon 上用 Rosetta, 可考虑原生 arm64 更快。"
  fi
  trans=$(sysctl -n sysctl.proc_translated 2>/dev/null || echo 0)
  echo "proc_translated(Rosetta): $trans  (1=Rosetta)"
  if command -v csrutil >/dev/null 2>&1; then
    echo "$(csrutil status)"
  else
    echo "csrutil 不可用, 跳过 SIP 检查"
  fi
  need xcode-select; need brew
  xcode-select -p || { err "请先装 Command Line Tools: xcode-select --install"; exit 1; }
  echo "磁盘可用: $(df -h / | awk 'NR==2{print $4}')"
  echo "CPUs: $(sysctl -n hw.ncpu)  RAM: $(sysctl -n hw.memsize | awk '{printf "%.0fGB",$1/1073741824}')"
}

# ---------------------------------------------------------------------------
phase1_deps() {
  log "Phase 1: 系统依赖 (brew install) — 最长步骤之一"
  brew doctor || echo "brew doctor 有警告, 继续(记录在案)"
  brew install asio assimp bison bullet cmake console_bridge cppcheck \
    cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
    pyqt@5 python qt@5 sip spdlog tinyxml2
}

# ---------------------------------------------------------------------------
phase2_python() {
  log "Phase 2: Python $PY venv"
  if ! brew list "$PY" >/dev/null 2>&1; then
    brew install "$PY"
  fi
  local pybin
  pybin="$(brew --prefix "$PY")/bin/python3.11"
  [ -x "$pybin" ] || pybin="$(brew --prefix "$PY")/bin/python3"
  echo "使用: $pybin  →  $("$pybin" --version)"

  [ -d "$WS/.venv" ] && rm -rf "$WS/.venv"
  "$pybin" -m venv "$WS/.venv"
  # 让后续命令都走 venv 的 python
  # shellcheck disable=SC1091
  source "$WS/.venv/bin/activate"
  echo "venv python: $(command -v python3)  $(python3 --version)"

  log "Phase 2b: 升级 pip + wheel"
  python3 -m pip install --upgrade pip wheel
}

# ---------------------------------------------------------------------------
phase3_pydeps() {
  log "Phase 3: 安装 Python 依赖 (官方 pin)"
  if [ -z "${VIRTUAL_ENV:-}" ]; then
    # shellcheck disable=SC1091
    source "$WS/.venv/bin/activate"
  fi
  # 1) 主批量: 必须去掉 --global-option/--config-settings！
  #    否则 pip 强制源码编译 cryptography/lxml 等(新版 cryptography 是 Rust 写的,
  #    会去 static.rust-lang.org 拉 rustup, 网络抖动即崩)。加该选项的锅。
  #    去掉后走预编译 wheel, 免 Rust。
  python3 -m pip install -U \
    argcomplete catkin_pkg colcon-common-extensions coverage \
    cryptography empy==3.3.4 flake8 flake8-blind-except==0.1.1 flake8-builtins \
    flake8-class-newline flake8-comprehensions flake8-deprecated \
    flake8-docstrings flake8-import-order flake8-quotes \
    importlib-metadata jsonschema lark==1.1.1 lxml matplotlib mock \
    mypy==0.931 netifaces nose pep8 psutil pydocstyle pydot \
    pyparsing==2.4.7 pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool
  # 2) pygraphviz 无 wheel, 必须用 brew 的 graphviz 头文件源码编译, 单独装
  python3 -m pip install -U \
    --config-settings="--global-option=build_ext" \
    --config-settings="--global-option=-I$(brew --prefix graphviz)/include" \
    --config-settings="--global-option=-L$(brew --prefix graphviz)/lib" \
    pygraphviz
}

# ---------------------------------------------------------------------------
phase4_env() {
  log "Phase 4: 环境变量写入 shell 配置"
  local rc="${ZDOTDIR:-$HOME}/.zshrc"
  [ -f "$rc" ] || rc="$HOME/.zshenv"
  touch "$rc"
  # 备份一次
  [ -f "$rc.bak.ros2" ] || cp "$rc" "$rc.bak.ros2"
  local line
  line='export OPENSSL_ROOT_DIR=$(brew --prefix openssl)'
  grep -qF "$line" "$rc" || echo "$line" >> "$rc"
  # brew bin 确保在 PATH (官方要求)
  grep -qF 'eval "$($(brew --prefix)/bin/brew shellenv)"' "$rc" \
    || { command -v brew >/dev/null && echo 'eval "$($(brew --prefix)/bin/brew shellenv)"' >> "$rc"; }
  # 激活 ROS2 环境(可选, 便于登录即用)
  export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$(brew --prefix qt@5)"
  export PATH="$(brew --prefix qt@5)/bin:$PATH"
  echo "# ROS2 Jazzy" >> "$rc"
  echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:$(brew --prefix qt@5)" >> "$rc"
  echo "export PATH=\$(brew --prefix qt@5)/bin:\$PATH" >> "$rc"
  echo "[ -f $WS/install/setup.zsh ] && source $WS/install/setup.zsh" >> "$rc"
  echo "已写入 $rc (备份在 $rc.bak.ros2)"
}

# ---------------------------------------------------------------------------
phase5_source() {
  log "Phase 5: 下载 ROS 2 源码 (vcs import)"
  if [ -z "${VIRTUAL_ENV:-}" ]; then
    # shellcheck disable=SC1091
    source "$WS/.venv/bin/activate"
  fi
  mkdir -p "$WS/src"
  cd "$WS"
  if [ ! -f "$WS/ros2.repos" ]; then
    curl -sSLo "$WS/ros2.repos" \
      https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
  fi
  vcs import --input "$WS/ros2.repos" "$WS/src"
  echo "共拉取仓库: $(find "$WS/src" -maxdepth 1 -mindepth 1 -type d | wc -l | tr -d ' ')"
}

# ---------------------------------------------------------------------------
phase6_build() {
  log "Phase 6: colcon 构建 (symlink-install, JOBS=$JOBS)"
  if [ -z "${VIRTUAL_ENV:-}" ]; then
    # shellcheck disable=SC1091
    source "$WS/.venv/bin/activate"
  fi
  cd "$WS"
  export CMAKE_POLICY_VERSION_MINIMUM=3.5
  export OPENSSL_ROOT_DIR="$(brew --prefix openssl)"
  export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$(brew --prefix qt@5)"
  export PATH="$(brew --prefix qt@5)/bin:$PATH"
  # 强制 find_package(Python3)/ament 一律用 venv 的 3.11, 避免匹配到 pyqt@5 顺带装的
  # python@3.14 框架(没有 catkin_pkg, 会让 tf2_py 等 python 包配置失败)。
  export Python3_EXECUTABLE="$WS/.venv/bin/python3"
  export Python_EXECUTABLE="$WS/.venv/bin/python3"
  export PYTHON_EXECUTABLE="$WS/.venv/bin/python3"

  colcon build --symlink-install \
    --event-handlers console_direct+ \
    --executor parallel \
    --parallel-workers "$JOBS" \
    --cmake-args \
      "-DPython3_EXECUTABLE=$WS/.venv/bin/python3" \
      "-DPython_EXECUTABLE=$WS/.venv/bin/python3" \
      "-DPYTHON_EXECUTABLE=$WS/.venv/bin/python3" \
    --packages-skip-by-dep python_qt_binding
  echo "构建完成。开始冒烟测试..."
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
}

# ---------------------------------------------------------------------------
phase7_smoke() {
  log "Phase 7: 冒烟测试"
  if [ -z "${VIRTUAL_ENV:-}" ]; then
    # shellcheck disable=SC1091
    source "$WS/.venv/bin/activate"
  fi
  [ -f "$WS/install/setup.bash" ] || { err "没有 setup.bash, 构建可能未完成"; exit 1; }
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  echo "ros2 版本: $(ros2 --help >/dev/null 2>&1 && echo OK || echo 未找到)"
  command -v ros2 >/dev/null 2>&1 || { err "ros2 命令不可用, 检查 setup.zsh 已 source"; exit 1; }
  echo "列表: $(ros2 pkg list 2>/dev/null | head -5 | tr '\n' ' ')..."
  ( ros2 run demo_nodes_cpp talker >"$LOG_DIR/talker.log" 2>&1 & echo $! >"$LOG_DIR/talker.pid" )
  sleep 5
  if grep -q "Publisher count is 1" "$LOG_DIR/talker.log"; then
    echo "✅ talker 已发布 (Publisher count is 1)"
  else
    echo "⚠️  talker 输出:"; tail -15 "$LOG_DIR/talker.log"
  fi
  kill "$(cat "$LOG_DIR/talker.pid")" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
main() {
  local phases; phases="${*:-0 1 2 3 4 5 6 7}"
  for p in $phases; do
    case "$p" in
      0) phase0_preflight ;;
      1) phase1_deps ;;
      2) phase2_python ;;
      3) phase3_pydeps ;;
      4) phase4_env ;;
      5) phase5_source ;;
      6) phase6_build ;;
      7) phase7_smoke ;;
      *) err "未知阶段: $p"; exit 1 ;;
    esac
  done
  log "完成。下一步见 runbook.md 常见问题/重开 SIP。"
}
main "$@"
