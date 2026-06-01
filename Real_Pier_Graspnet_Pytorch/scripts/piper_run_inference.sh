#!/bin/bash
# 在 PGX 感知容器内执行单步感知
# 用法: ./piper_run_inference.sh [target_query] [remote_ip]
# 示例: ./piper_run_inference.sh bottle 192.168.12.101
#       ./piper_run_inference.sh cup 192.168.20.1

set -e

CONTAINER_NAME="piper_inference"
TARGET_QUERY="${1:-bottle}"
REMOTE_IP="${2:-192.168.12.101}"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[ERROR] 感知容器 '${CONTAINER_NAME}' 未运行。请先运行 piper_start_inference.sh"
    exit 1
fi

echo "执行单步感知: target='${TARGET_QUERY}', remote_ip='${REMOTE_IP}'"
echo "-------------------------------------------"

docker exec -it "${CONTAINER_NAME}" bash -c "\
cd /root/contact_graspnet && \
source /opt/ros/humble/setup.bash && \
python3 tools/inference_isaac_sim.py \
    --ckpt_dir /root/contact_graspnet/checkpoints/contact_graspnet \
    --remote-ip ${REMOTE_IP} \
    --sensor-port 5555 \
    --grasp-port 5556 \
    --z-range '[0.05,1.5]' \
    --use-segmap \
    --segmentation-source sam \
    --sam-model-type vit_h \
    --sam-checkpoint /models/sam_vit_h_4b8939.pth \
    --target-query '${TARGET_QUERY}' \
    --target-selector open_clip \
    --local-regions \
    --filter-grasps \
    --filter-grasps-threshold 0.02 \
    --segment-grasp-only \
    --forward-passes 3 \
    --min-grasp-score 0.15 \
    --target-min-points 1000 \
    --target-stable-frames 1 \
    --max-contact-distance-to-target 0.02 \
    --target-lock \
    --max-pregrasp-offset 0.06 \
    --max-retreat-offset 0.08 \
    --execute-best-grasp \
    --execution-base-position-world '[0,0,0]' \
    --execution-base-yaw-deg 0 \
    --execution-base-workspace-bounds '' \
    --end-pose-is-camera-pose \
    --debug-save-dir /root/contact_graspnet/debug/segmentation"