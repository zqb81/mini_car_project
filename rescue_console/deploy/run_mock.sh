#!/usr/bin/env bash
# =============================================================================
# 模拟模式快速启动（不依赖 ROS、不注册服务，用于部署链路自检）
#
# 用法（目标机或开发机均可）：
#   cd ~/mini_car_ws/rescue_console/deploy
#   ./run_mock.sh              # 使用已有 venv；不存在则自动创建
#
# 自检通过标志：浏览器打开 http://<本机IP>:8000 能看到地图并遥控小车移动。
# =============================================================================
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -x "$APP_DIR/venv/bin/python" ]]; then
    echo "==> venv 不存在，先安装依赖"
    python3 -m venv "$APP_DIR/venv"
    "$APP_DIR/venv/bin/pip" install --upgrade pip -q
    "$APP_DIR/venv/bin/pip" install -r "$APP_DIR/server/requirements.txt"
fi

echo "==> 模拟模式启动：http://0.0.0.0:8000 （Ctrl+C 退出）"
cd "$APP_DIR/server"
RESCUE_BRIDGE=mock exec "$APP_DIR/venv/bin/python" -m uvicorn app:app \
    --host 0.0.0.0 --port 8000
