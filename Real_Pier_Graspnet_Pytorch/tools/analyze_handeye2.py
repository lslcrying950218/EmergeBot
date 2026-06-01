#!/usr/bin/env python3
"""Analyze hand-eye calibration parameters."""
import numpy as np
from scipy.spatial.transform import Rotation as R

# Hand-eye calibration parameters
cam_pos_gripper = np.array([-0.0763, 0.0039, 0.035])
cam_quat_gripper = np.array([-0.120, 0.124, -0.697, 0.696])

# Build transformation matrix T_cam_gripper (camera pose in gripper frame)
T_cam_gripper = np.eye(4)
T_cam_gripper[:3, :3] = R.from_quat(cam_quat_gripper).as_matrix()
T_cam_gripper[:3, 3] = cam_pos_gripper

print("=" * 60)
print("Hand-eye calibration analysis")
print("=" * 60)

print("\nT_cam_gripper (camera pose in gripper frame):")
print("Position:", np.round(T_cam_gripper[:3, 3], 4).tolist())
print("Rotation:")
print(np.round(T_cam_gripper[:3, :3], 4))

# Check camera axes in gripper frame
print("\nCamera axes direction in gripper frame:")
print("  Camera X (right):", np.round(T_cam_gripper[:3, 0], 3).tolist())
print("  Camera Y (down):", np.round(T_cam_gripper[:3, 1], 3).tolist())
print("  Camera Z (forward):", np.round(T_cam_gripper[:3, 2], 3).tolist())

# Euler angles
euler = R.from_quat(cam_quat_gripper).as_euler('xyz', degrees=True)
print("\nEuler angles (XYZ):", np.round(euler, 2).tolist())

print("\n" + "=" * 60)
print("Physical interpretation")
print("=" * 60)
print("""
Camera position in gripper frame: [-0.076, 0.004, 0.035]
  - X=-0.076: 7.6cm behind gripper center
  - Y=+0.004: 0.4cm to the left
  - Z=+0.035: 3.5cm above gripper center

Camera Z-axis (forward) in gripper: [0.340, -0.006, 0.940]
  - Mainly points in +Z direction (UPWARD in gripper frame)
  - With component in +X (forward)

This suggests camera is mounted pointing UPWARD, not forward!
But you said camera is mounted pointing FORWARD with slight downward tilt.
""")