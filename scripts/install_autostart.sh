#!/usr/bin/env bash
set -euo pipefail

# 这些变量都可以通过环境变量或命令行参数覆盖。
SERVICE_NAME="${SERVICE_NAME:-lubancat-apriltag}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${CONFIG_PATH:-${PROJECT_DIR}/config/lubancat0n.json}"
RUN_AS_USER="${RUN_AS_USER:-${SUDO_USER:-$(id -un)}}"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

usage() {
  # 打印安装脚本的用法。
  cat <<USAGE
Usage: sudo bash scripts/install_autostart.sh [options]

Options:
  --config PATH        Config file path, default: ${CONFIG_PATH}
  --user USER          Linux user for the service, default: ${RUN_AS_USER}
  --service-name NAME  systemd service name, default: ${SERVICE_NAME}
  -h, --help           Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  # 解析可选参数：配置文件路径、运行用户、服务名。
  case "$1" in
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --user)
      RUN_AS_USER="$2"
      shift 2
      ;;
    --service-name)
      SERVICE_NAME="$2"
      SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$(id -u)" -ne 0 ]]; then
  # 写 /etc/systemd/system 和启用服务都需要 root 权限。
  echo "please run with sudo: sudo bash scripts/install_autostart.sh" >&2
  exit 1
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  # 开机服务依赖真实配置文件；没有配置时不创建服务。
  echo "config not found: ${CONFIG_PATH}" >&2
  echo "copy config/example_config.json to config/lubancat0n.json and edit camera/mavlink settings first" >&2
  exit 2
fi

chmod +x "${PROJECT_DIR}/scripts/run_landing_target.sh"

# 生成 systemd 服务文件：开机启动、崩溃自动重启、附加摄像头/串口权限组。
cat > "${SERVICE_FILE}" <<SERVICE
[Unit]
Description=AprilTag LANDING_TARGET sender
After=multi-user.target systemd-udev-settle.service
Wants=multi-user.target systemd-udev-settle.service

[Service]
Type=simple
User=${RUN_AS_USER}
WorkingDirectory=${PROJECT_DIR}
Environment=LANDING_TARGET_CONFIG=${CONFIG_PATH}
Environment=PYTHONUNBUFFERED=1
Environment=MAVLINK20=1
Environment=STARTUP_DELAY=8
Environment=DEVICE_WAIT_TIMEOUT=60
SupplementaryGroups=video dialout
ExecStart=${PROJECT_DIR}/scripts/run_landing_target.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICE

# 重新加载 systemd，设置开机自启动，并立即启动一次服务。
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
systemctl restart "${SERVICE_NAME}.service"

cat <<DONE
Installed and started ${SERVICE_NAME}.service

Check status:
  systemctl status ${SERVICE_NAME}.service

Watch logs:
  journalctl -u ${SERVICE_NAME}.service -f

Stop autostart:
  systemctl disable --now ${SERVICE_NAME}.service
DONE
