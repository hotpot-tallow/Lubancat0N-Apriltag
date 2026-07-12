#!/usr/bin/env bash
set -euo pipefail

# 找到脚本所在目录和项目根目录，保证从任意路径执行都能定位项目。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认使用 config/lubancat0n.json，也允许通过环境变量或第一个参数覆盖。
CONFIG_PATH="${LANDING_TARGET_CONFIG:-${PROJECT_DIR}/config/lubancat0n.json}"
PYTHON_BIN="${PYTHON_BIN:-}"
STARTUP_DELAY="${STARTUP_DELAY:-8}"
DEVICE_WAIT_TIMEOUT="${DEVICE_WAIT_TIMEOUT:-60}"

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

wait_for_device() {
  local path="$1"
  local label="$2"
  local deadline=$((SECONDS + DEVICE_WAIT_TIMEOUT))

  if [[ -z "${path}" ]]; then
    return 0
  fi

  echo "waiting for ${label}: ${path}"
  while [[ ! -e "${path}" ]]; do
    if (( SECONDS >= deadline )); then
      echo "timeout waiting for ${label}: ${path}" >&2
      return 1
    fi
    sleep 1
  done
  echo "${label} ready: ${path}"
}

if [[ "${STARTUP_DELAY}" != "0" ]]; then
  # 飞控排针供电时，系统服务可能比摄像头、串口和飞控启动得更早；先等电源和设备枚举稳定。
  echo "startup delay ${STARTUP_DELAY}s before opening camera and mavlink"
  sleep "${STARTUP_DELAY}"
fi

mapfile -t DEVICE_PATHS < <("${PYTHON_BIN}" - "${CONFIG_PATH}" <<'PY'
import json
import re
import sys

with open(sys.argv[1], "r", encoding="utf-8") as fp:
    cfg = json.load(fp)

camera_device = cfg["camera"].get("device", "")
camera_backend = str(cfg["camera"].get("backend", "")).lower()
mavlink_device = str(cfg["mavlink"].get("connection", ""))

if camera_backend in ("picamera2", "picam2"):
    # Picamera2 通过 libcamera/media devices 自动发现相机，不对应固定的 /dev/videoN。
    camera_path = ""
elif isinstance(camera_device, int):
    camera_path = f"/dev/video{camera_device}"
else:
    camera_text = str(camera_device)
    match = re.search(r"device=(/dev/[^ !]+)", camera_text)
    camera_path = match.group(1) if match else (camera_text if camera_text.startswith("/dev/") else "")

print(camera_path)
print(mavlink_device if mavlink_device.startswith("/dev/") else "")
PY
)

wait_for_device "${DEVICE_PATHS[0]:-}" "camera"
wait_for_device "${DEVICE_PATHS[1]:-}" "mavlink"

cd "${PROJECT_DIR}"
# PYTHONPATH 指向 src，确保不安装包也能 import lubancat_apriltag。
export PYTHONPATH="${PROJECT_DIR}/src"
export PYTHONUNBUFFERED=1
export MAVLINK20=1

# 用 exec 替换当前 shell，systemd 看到的主进程就是 Python 程序。
exec "${PYTHON_BIN}" "${PROJECT_DIR}/tools/landing_target.py" --config "${CONFIG_PATH}"
