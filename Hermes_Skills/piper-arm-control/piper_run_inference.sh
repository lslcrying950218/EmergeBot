#!/bin/bash
# 在 PGX 上一键播放机械臂轨迹动画
# 此脚本会 SSH 到 Jetson 并执行轨迹重放
#
# Jetson: amov@192.168.12.101 密码: amov
# 用法: ./piper_play_trajectory.sh

set -e

JETSON_USER="amov"
JETSON_IP="192.168.12.101"
JETSON_PASS="amov"
CONTAINER_NAME="piper_control"
PLAY_TIMES="1"
PLAY_SPEED="1.0"
MOVE_SPD_RATE="100"
CSV_PATH=""

# 检查 sshpass
if ! command -v sshpass &>/dev/null; then
    echo "[ERROR] sshpass 未安装。请运行: sudo apt install sshpass"
    exit 1
fi

ssh_cmd() {
    sshpass -p "${JETSON_PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_IP}" "$1"
}

# === Step 1: 检查连接 ===
echo "=== 步骤 1: 连接 Jetson (${JETSON_IP}) ==="

if ! ssh_cmd "true"; then
    echo "[ERROR] 无法连接到 Jetson (${JETSON_IP})。请检查网络。"
    exit 1
fi
echo "[OK] Jetson 连接成功。"

# === Step 2: 检查 CAN ===
echo "=== 步骤 2: 检查 CAN 接口 ==="

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

# === Step 3: 检查 Docker 容器 ===
echo "=== 步骤 3: 检查 Docker 容器 ==="

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

# === Step 4: 查看当前状态 ===
echo "=== 步骤 4: 查看机械臂状态 ==="

ssh_cmd "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && source /root/piper_ros_ws/install/setup.bash && cd /root/contact_graspnet/tools && python3 trajectory_record_replay.py --mode status'"

# === Step 5: 执行轨迹重放 ===
echo "=== 步骤 5: 播放机械臂轨迹 ==="
echo ""
echo "参数:"
echo "  Jetson IP: ${JETSON_IP}"
echo "  播放次数: ${PLAY_TIMES} (0=无限循环)"
echo "  播放速度: ${PLAY_SPEED}x"
echo "  运动速度: ${MOVE_SPD_RATE}%"

# 构建命令参数
CMD_ARGS="--mode replay --play-times ${PLAY_TIMES} --play-speed ${PLAY_SPEED} --move-spd-rate ${MOVE_SPD_RATE} --auto"
if [ -n "$CSV_PATH" ]; then
    CMD_ARGS="${CMD_ARGS} --csv-path ${CSV_PATH}"
fi

echo ""
echo "按 Ctrl+C 可提前终止播放..."
echo ""

# 使用非交互模式执行（因为 sshpass 不支持 TTY）
ssh_cmd "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && source /root/piper_ros_ws/install/setup.bash && cd /root/contact_graspnet/tools && python3 trajectory_record_replay.py ${CMD_ARGS}'"

echo ""
echo "[OK] 轨迹播放完成。"
