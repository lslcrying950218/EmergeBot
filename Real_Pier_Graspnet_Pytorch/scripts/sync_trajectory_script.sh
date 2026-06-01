#!/bin/bash
# 将本地修改的 trajectory_record_replay.py 同步到 Jetson Docker 容器中

JETSON_USER="amov"
JETSON_IP="${1:-192.168.12.101}"
JETSON_PASS="amov"
CONTAINER_NAME="piper_control"

LOCAL_FILE="/home/emergeos/Share_pgx/Piper_grasp_control/piper_ros/contact_graspnet_pytorch/tools/trajectory_record_replay.py"
REMOTE_PATH="/root/contact_graspnet/tools/trajectory_record_replay.py"

echo "=== 同步 trajectory_record_replay.py 到 Jetson (${JETSON_IP}) ==="

# 检查本地文件
if [ ! -f "$LOCAL_FILE" ]; then
    echo "[ERROR] 本地文件不存在: $LOCAL_FILE"
    exit 1
fi

# 复制到 Jetson /tmp
echo "[STEP 1] 复制文件到 Jetson..."
sshpass -p "${JETSON_PASS}" scp -o StrictHostKeyChecking=no "$LOCAL_FILE" "${JETSON_USER}@${JETSON_IP}:/tmp/"

# 从 Jetson host 复制到 Docker 容器
echo "[STEP 2] 复制文件到 Docker 容器..."
sshpass -p "${JETSON_PASS}" ssh "${JETSON_USER}@${JETSON_IP}" "docker cp /tmp/trajectory_record_replay.py ${CONTAINER_NAME}:${REMOTE_PATH}"

echo "[OK] 同步完成"
echo ""
echo "现在可以运行: ./scripts/piper_play_trajectory.sh"
