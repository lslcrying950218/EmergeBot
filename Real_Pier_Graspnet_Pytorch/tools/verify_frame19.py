#!/usr/bin/env python3
"""Verify coordinate transformation consistency."""
import numpy as np
from scipy.spatial.transform import Rotation as R

print("=" * 70)
print("Frame 19 Coordinate Analysis")
print("=" * 70)

# Input data
optical = np.array([-0.098, -0.2204, 0.6545])
camera_pos = np.array([0.1104, 0.0138, 0.2591])
camera_rot = np.array([
    [0.1001, -0.5043, 0.8577],
    [-0.9947, -0.0298, 0.0985],
    [-0.0241, -0.863, -0.5046]
])
actual_target = np.array([0.394, 0.201, 0.074])

# Calculate world position
world_pos = camera_pos + camera_rot @ optical

print(f"\n光学坐标: {np.round(optical, 4).tolist()}")
print(f"相机位置: {np.round(camera_pos, 4).tolist()}")
print(f"计算的世界坐标: {np.round(world_pos, 4).tolist()}")
print(f"实际目标: {np.round(actual_target, 4).tolist()}")
print(f"误差: {np.round(world_pos - actual_target, 4).tolist()}")

# Analyze camera orientation
print("\n" + "=" * 70)
print("Camera Orientation Analysis")
print("=" * 70)

print(f"\n相机各轴在世界坐标系中的方向:")
print(f"  X轴(右): {np.round(camera_rot[:, 0], 4).tolist()}")
print(f"  Y轴(下): {np.round(camera_rot[:, 1], 4).tolist()}")
print(f"  Z轴(前): {np.round(camera_rot[:, 2], 4).tolist()}")

# Decompose contributions
print(f"\n各光学轴对世界坐标的贡献:")
contrib_x = optical[0] * camera_rot[:, 0]
contrib_y = optical[1] * camera_rot[:, 1]
contrib_z = optical[2] * camera_rot[:, 2]
print(f"  optical_x ({optical[0]:.4f}) -> {np.round(contrib_x, 4).tolist()}")
print(f"  optical_y ({optical[1]:.4f}) -> {np.round(contrib_y, 4).tolist()}")
print(f"  optical_z ({optical[2]:.4f}) -> {np.round(contrib_z, 4).tolist()}")

# Calculate actual vector from camera to target
actual_vector = actual_target - camera_pos
print(f"\n从相机到实际目标的真实向量:")
print(f"  方向: {np.round(actual_vector, 4).tolist()}")
print(f"  距离: {np.linalg.norm(actual_vector):.4f}m")

# Calculate expected optical coordinates based on actual target
# If world = R @ optical + t, then optical = R^T @ (world - t)
expected_optical = camera_rot.T @ actual_vector
print(f"\n如果坐标转换正确，光学坐标应该是:")
print(f"  {np.round(expected_optical, 4).tolist()}")
print(f"  实际检测到的: {np.round(optical, 4).tolist()}")
print(f"  差异: {np.round(optical - expected_optical, 4).tolist()}")

# Check hand-eye calibration
print("\n" + "=" * 70)
print("Hand-eye Calibration Check")
print("=" * 70)

hand_eye_pos = np.array([-0.0763, 0.0039, 0.035])
hand_eye_quat = np.array([-0.120, 0.124, -0.697, 0.696])
hand_eye_rot = R.from_quat(hand_eye_quat).as_matrix()

print(f"\n手眼标定参数:")
print(f"  位置: {hand_eye_pos.tolist()}")
print(f"  四元数: {hand_eye_quat.tolist()}")
print(f"  相机Z轴在gripper中的方向: {np.round(hand_eye_rot[:, 2], 4).tolist()}")

# Check if camera_pose_publisher formula is correct
print(f"\ncamera_pose_publisher 公式验证:")
print(f"  cam_pos_world = gripper_pos + gripper_rot @ cam_pos_gripper")
print(f"  cam_rot_world = gripper_rot @ cam_rot_gripper")
print(f"\n这个公式假设 cam_pos_gripper 是相机原点在gripper坐标系中的位置")
print(f"即: 手眼标定给出的是 T_camera_gripper")

# Alternative interpretation
print(f"\n另一种解释:")
print(f"如果手眼标定给出的是 T_gripper_camera:")
print(f"  cam_pos_world = gripper_pos + gripper_rot @ inv(hand_eye_rot) @ (-hand_eye_pos)")
T_gc = np.eye(4)
T_gc[:3, :3] = hand_eye_rot
T_gc[:3, 3] = hand_eye_pos
T_cg = np.linalg.inv(T_gc)
cam_pos_in_gripper_alt = T_cg[:3, 3]
print(f"  相机在gripper中的位置(另一解释): {np.round(cam_pos_in_gripper_alt, 4).tolist()}")

