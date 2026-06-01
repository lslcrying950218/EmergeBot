#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R

# Test data from user
optical = np.array([0.2721, -0.1570, 0.3639])
camera_pos = np.array([0.1349, -0.0035, 0.2068])
camera_rot = np.array([
    [-0.0575, -0.7905, 0.6098],
    [-0.9981, 0.0579, -0.0191],
    [-0.0202, -0.6098, -0.7923]
])
actual_target = np.array([0.422, -0.165, 0.056])

print("=" * 60)
print("Coordinate Transformation Analysis")
print("=" * 60)

# Current calculation
world_pos = camera_pos + camera_rot @ optical
print(f"\nCurrent calculation:")
print(f"  Optical: {np.round(optical, 4).tolist()}")
print(f"  Camera pos: {np.round(camera_pos, 4).tolist()}")
print(f"  Calculated world: {np.round(world_pos, 4).tolist()}")
print(f"  Actual target: {np.round(actual_target, 4).tolist()}")
print(f"  Error: {np.round(world_pos - actual_target, 4).tolist()}")

# Camera rotation matrix axes
print(f"\nCamera rotation matrix axes:")
print(f"  X axis (right): {np.round(camera_rot[:, 0], 4).tolist()}")
print(f"  Y axis (down): {np.round(camera_rot[:, 1], 4).tolist()}")
print(f"  Z axis (forward): {np.round(camera_rot[:, 2], 4).tolist()}")

# Try optical-to-camera transform
print("\n" + "=" * 60)
print("Testing optical->camera frame transform")
print("=" * 60)
R_optical_to_camera = np.array([
    [-1, 0, 0],
    [0, -1, 0],
    [0, 0, 1]
])
print("Standard transform (180 deg around Z):")
print(R_optical_to_camera)

optical_in_camera = R_optical_to_camera @ optical
print(f"\nOptical coords after transform: {np.round(optical_in_camera, 4).tolist()}")

camera_rot_corrected = camera_rot @ R_optical_to_camera
world_pos_corrected = camera_pos + camera_rot_corrected @ optical
print(f"Corrected world coords: {np.round(world_pos_corrected, 4).tolist()}")
print(f"Corrected error: {np.round(world_pos_corrected - actual_target, 4).tolist()}")

# Analyze each axis contribution
print("\n" + "=" * 60)
print("Axis contribution analysis")
print("=" * 60)
contrib_x = optical[0] * camera_rot[:, 0]
contrib_y = optical[1] * camera_rot[:, 1]
contrib_z = optical[2] * camera_rot[:, 2]
print(f"Optical X (0.272) contribution: {np.round(contrib_x, 4).tolist()}")
print(f"Optical Y (-0.157) contribution: {np.round(contrib_y, 4).tolist()}")
print(f"Optical Z (0.364) contribution: {np.round(contrib_z, 4).tolist()}")

# Hand-eye calibration check
print("\n" + "=" * 60)
print("Hand-eye calibration matrix")
print("=" * 60)
hand_eye_translation = np.array([-0.0763, 0.0039, 0.035])
hand_eye_quat_xyzw = np.array([-0.120, 0.124, -0.697, 0.696])

T_gripper_to_camera = np.eye(4)
T_gripper_to_camera[:3, :3] = R.from_quat(hand_eye_quat_xyzw).as_matrix()
T_gripper_to_camera[:3, 3] = hand_eye_translation

print("Rotation part:")
print(np.round(T_gripper_to_camera[:3, :3], 4))
print("\nCamera axes in gripper frame:")
rot = T_gripper_to_camera[:3, :3]
print(f"  Camera X: {np.round(rot[:, 0], 4).tolist()}")
print(f"  Camera Y: {np.round(rot[:, 1], 4).tolist()}")
print(f"  Camera Z: {np.round(rot[:, 2], 4).tolist()}")

# Check if hand-eye calibration rotation is close to 180 degree rotation
euler = R.from_matrix(rot).as_euler('xyz', degrees=True)
print(f"\nHand-eye Euler angles (XYZ): {np.round(euler, 2).tolist()}")

