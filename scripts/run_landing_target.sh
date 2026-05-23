#!/usr/bin/env bash
set -euo pipefail

# 找到脚本所在目录和项目根目录，保证从任意路径执行都能定位项目。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认使用 config/lubancat0n.json，也允许通过环境变量或第一个参数覆盖。
CONFIG_PATH="${LANDING_TARGET_CONFIG:-${PROJECT_DIR}/config/lubancat0n.json}"
PYTHON_BIN="${PYTHON_BIN:-}"

if [[ $# -gt 0 ]]; then
  CONFIG_PATH="$1"
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  # 配置文件不存在时直接退出，避免开机服务反复以错误参数运行。
  echo "config not found: ${CONFIG_PATH}" >&2
  echo "copy config/example_config.json to config/lubancat0n.json and edit it first" >&2
  exit 2
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  # 优先使用项目里的虚拟环境；没有虚拟环境时退回系统 python3。
  if [[ -x "${PROJECT_DIR}/venv/bin/python" ]]; then
    PYTHON_BIN="${PROJECT_DIR}/venv/bin/python"
  else
    PYTHON_BIN="python3"
  fi
fi

cd "${PROJECT_DIR}"
# PYTHONPATH 指向 src，确保不安装包也能 import lubancat_apriltag。
export PYTHONPATH="${PROJECT_DIR}/src"
export PYTHONUNBUFFERED=1
export MAVLINK20=1

# 用 exec 替换当前 shell，systemd 看到的主进程就是 Python 程序。
exec "${PYTHON_BIN}" "${PROJECT_DIR}/tools/landing_target.py" --config "${CONFIG_PATH}"
