#!/usr/bin/env bash
# =============================================================================
# 救援控制台网关 - 目标机开机自启配置脚本（可选）
#
# 适用环境：树莓派 / x86 主机 + Ubuntu 22.04 + ROS2 Humble。
#
# 说明：本项目以源码方式交付，平时直接跑源码即可（README 第 5.1 节）。
#       本脚本只在“希望开机自启/后台常驻”时执行，它会：创建 venv、
#       安装 requirements.txt、注册 systemd 服务。
#
# 前置条件：
#   1. 本仓库已按标准布局克隆到 ~/mini_car_ws（见 docs/AGENT_HANDOFF.md 第 2 节）
#   2. 当前用户可执行 sudo（仅用于 apt 与 systemd 安装，需要交互输密码）
#
# 用法：
#   cd ~/mini_car_ws/rescue_console/deploy
#   ./install.sh            # 注册 systemd 服务（接真实 ROS2 话题）
#
# 产物：
#   ~/mini_car_ws/rescue_console/venv            Python 虚拟环境（Web 依赖）
#   /etc/systemd/system/rescue-console.service   systemd 服务（开机自启）
#   服务地址：http://<目标机IP>:8000
# =============================================================================
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="$(whoami)"
HOME_DIR="$(eval echo "~$USER_NAME")"

echo "==> 部署目录: $APP_DIR"
echo "==> 运行用户: $USER_NAME ($HOME_DIR)"
echo "==> 桥接模式: ros（固定，接真实 ROS2 话题）"

# 1. 系统依赖：python3-venv 用于创建隔离环境，避免污染系统 pip。
#    Ubuntu 22.04 无 PEP 668 限制，但 venv 仍是最干净的方案。
if ! dpkg -s python3-venv >/dev/null 2>&1; then
    echo "==> 安装 python3-venv（需要 sudo）"
    sudo apt-get update -qq
    sudo apt-get install -y python3-venv
fi

PY_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
echo "==> Python 版本: $PY_VERSION（要求 >= 3.10）"
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' || {
    echo "错误：Python >= 3.10 必需，当前 $PY_VERSION"; exit 1;
}

# 2. Web 依赖虚拟环境（rclpy 不装进 venv，由系统 ROS 环境提供）。
#    仅当希望开机自启时才需要本脚本；手工运行见 README 第 5.1 节。
echo "==> 创建虚拟环境并安装 Web 依赖"
python3 -m venv "$APP_DIR/venv"
"$APP_DIR/venv/bin/pip" install --upgrade pip -q
"$APP_DIR/venv/bin/pip" install -r "$APP_DIR/server/requirements.txt"

# 3. 生成 systemd 服务（模板占位符替换为实际路径与用户）
echo "==> 注册 systemd 服务"
sed -e "s|__USER__|$USER_NAME|g" \
    -e "s|__HOME__|$HOME_DIR|g" \
    -e "s|__APP_DIR__|$APP_DIR|g" \
    "$APP_DIR/deploy/rescue-console.service.template" \
    > /tmp/rescue-console.service
sudo cp /tmp/rescue-console.service /etc/systemd/system/rescue-console.service
rm -f /tmp/rescue-console.service

sudo systemctl daemon-reload
sudo systemctl enable rescue-console.service
sudo systemctl restart rescue-console.service

sleep 2
echo "==> 服务状态："
systemctl --no-pager --lines=5 status rescue-console.service || true

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo ""
echo "============================================================"
echo " 部署完成。浏览器访问：http://${IP:-<目标机IP>}:8000"
echo ""
echo " 常用命令："
echo "   systemctl status rescue-console       # 查看状态"
echo "   journalctl -u rescue-console -f       # 看日志"
echo "   sudo systemctl edit rescue-console    # 改 RESCUE_LASER_YAW_OFFSET 等环境变量"
echo "   sudo systemctl restart rescue-console"
echo ""
echo " 注意：本服务需 slam_navigation.launch.py 在运行，"
echo "       且首次接实车务必车轮悬空（见 rescue_console/README.md 安全约束）"
echo "============================================================"
