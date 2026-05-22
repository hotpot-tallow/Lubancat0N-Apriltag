#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="${LANDING_TARGET_CONFIG:-${PROJECT_DIR}/config/lubancat0n.json}"
PYTHON_BIN="${PYTHON_BIN:-}"

if [[ $# -gt 0 ]]; then
  CONFIG_PATH="$1"
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "config not found: ${CONFIG_PATH}" >&2
  echo "copy config/example_config.json to config/lubancat0n.json and edit it first" >&2
  exit 2
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x "${PROJECT_DIR}/venv/bin/python" ]]; then
    PYTHON_BIN="${PROJECT_DIR}/venv/bin/python"
  else
    PYTHON_BIN="python3"
  fi
fi

cd "${PROJECT_DIR}"
export PYTHONPATH="${PROJECT_DIR}/src"
export PYTHONUNBUFFERED=1
export MAVLINK20=1

exec "${PYTHON_BIN}" "${PROJECT_DIR}/tools/landing_target.py" --config "${CONFIG_PATH}"
