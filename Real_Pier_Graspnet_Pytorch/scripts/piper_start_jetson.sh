#!/bin/bash
# 在 Jetson 上检查 CAN、启动 Docker 容器、运行一键启动脚本
# Jetson: amov@192.168.12.101 密码: amov
# 用法: ./piper_start_jetson.sh [jetson_ip]
# 示例: ./piper_start_jetson.sh
#       ./piper_start_jetson.sh 192.168.20.1

set -e

JETSON_USER="amov"
JETSON_IP="${1:-192.168.12.101}"
JETSON_PASS="amov"
CONTAINER_NAME="piper_control"

# 检查 sshpass
if ! command -v sshpass &>/dev/null; then
    echo "[ERROR] sshpass 未安装。请运行: sudo apt install sshpass"
    exit 1
fi

ssh_cmd() {
    sshpass -p "${JETSON_PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_IP}" "$1"
}

# === Step 1: 检查 CAN ===
echo "=== 步骤 1: 检查 Jetson CAN 接口 (${JETSON_IP}) ==="

if ! ssh_cmd "true"; then
    echo "[ERROR] 无法连接到 Jetson (${JETSON_IP})。请检查网络。"
    exit 1
fi

CAN_STATUS=$(ssh_cmd "ip link show can0 2>/dev/null | grep 'state UP'" || true)
if [ -n "$CAN_STATUS" ]; then
    echo "[OK] CAN0 已启动 (state UP)。"
else
    echo "CAN0 未启动，正在设置..."
    ssh_cmd "echo '${JETSON_PASS}' | sudo -S ip link set can0 up type can bitrate 1000000 2>/dev/null"
    sleep 1
    CAN_STATUS=$(ssh_cmd "ip link show can0 2>/dev/null | grep 'state UP'" || true)
    if [ -n "$CAN_STATUS" ]; then
        echo "[OK] CAN0 已启动。"
    else
        echo "[ERROR] CAN0 启动失败。请检查硬件连接。"
        exit 1
    fi
fi

# === Step 2: 检查 Docker 容器 ===
echo "=== 步骤 2: 检查 Jetson Docker 容器 ==="

CONTAINER_RUNNING=$(ssh_cmd "docker ps --format '{{.Names}}'" | grep "^${CONTAINER_NAME}$" || true)
if [ -n "$CONTAINER_RUNNING" ]; then
    echo "[OK] 容器 '${CONTAINER_NAME}' 已在运行。"
else
    CONTAINER_EXISTS=$(ssh_cmd "docker ps -a --format '{{.Names}}'" | grep "^${CONTAINER_NAME}$" || true)
    if [ -n "$CONTAINER_EXISTS" ]; then
        echo "正在启动容器 '${CONTAINER_NAME}'..."
        ssh_cmd "docker start ${CONTAINER_NAME}"
        sleep 2
        echo "[OK] 容器已启动。"
    else
        echo "[ERROR] 容器 '${CONTAINER_NAME}' 在 Jetson 上不存在。"
        echo "请在 Jetson 上手动创建容器: docker run -d --name piper_control ..."
        exit 1
    fi
fi

# === Step 3: 运行一键启动脚本 ===
echo "=== 步骤 3: 在 Jetson 容器内运行一键启动脚本 ==="

# 先清理已有 Python 进程
ssh_cmd "docker exec ${CONTAINER_NAME} bash -c 'pkill -f python3 2>/dev/null || true'" || true
sleep 1

# 在容器内后台运行启动脚本
ssh_cmd "docker exec -d ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && source /root/piper_ros_ws/install/setup.bash && source /root/ros2_cyclonedds_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && bash /root/contact_graspnet/tools/start_all.sh > /tmp/start_all.log 2>&1'" || true

sleep 3

echo "[OK] 一键启动脚本已在后台运行。"
echo ""
echo "常用命令:"
echo "  查看日志: sshpass -p 'amov' ssh amov@${JETSON_IP} \"docker exec piper_control cat /tmp/start_all.log\""
echo "  停止脚本: sshpass -p 'amov' ssh amov@${JETSON_IP} \"docker exec piper_control pkill -f python3\""
echo "  停止容器: sshpass -p 'amov' ssh amov@${JETSON_IP} \"docker stop piper_control\""