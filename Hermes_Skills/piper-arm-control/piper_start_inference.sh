#!/bin/bash
# 启动 PGX 上的感知容器 (piper_inference)
# 若容器已运行则跳过，若已停止则重启，若不存在则创建

set -e

CONTAINER_NAME="piper_inference"

mkdir -p /home/emergeos/debug_segmentation

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[OK] 感知容器 '${CONTAINER_NAME}' 已在运行。"
    exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "感知容器已停止，正在重启..."
    docker start "${CONTAINER_NAME}"
    echo "[OK] 感知容器已启动。"
    exit 0
fi

echo "正在创建感知容器 '${CONTAINER_NAME}'..."
docker run -d --name piper_inference --gpus all --network host \
    -v /home/emergeos/Share_pgx/contact_graspnet_pytorch-main:/root/contact_graspnet \
    -v /home/emergeos/Share_pgx/seg_map/models/gsa_weights:/models:ro \
    -v /home/emergeos/Share_pgx/seg_map/models/huggingface:/root/.cache/huggingface \
    -v /home/emergeos/Share_pgx/debug_segmentation:/root/contact_graspnet/debug \
    piper_ros:humble_clip tail -f /dev/null

echo "[OK] 感知容器已创建并启动。"