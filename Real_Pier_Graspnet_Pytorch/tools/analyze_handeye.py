#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R

# Hand-eye calibration parameters (from file)
cam_pos = np.array([-0.0763, 0.0039, 0.0350])
cam_quat = np.array([-0.120, 0.124, -0.697, 0.696])

# Build T_camera_gripper (camera pose in gripper frame)
T_camera_gripper = np.eye(4)
T_camera_gripper[:3, :3] = R.from_quat(cam_quat).as_matrix()
T_camera_gripper[:3, 3] = cam_pos

print("=" * 60)
print("Analyzing hand-eye calibration parameters")
print("=" * 60)
print("\nGiven parameters:")
print(f"  Position: {cam_pos}")
print(f"  Quaternion (xyzw): {cam_quat}")

print("\nBuilt T_camera_gripper (camera pose in gripper frame):")
print("  Rotation:")
print(T_camera_gripper[:3, :3])
print("  Translation:", T_camera_gripper[:3, 3])

# Check camera axes in gripper frame
print("\nCamera axes in gripper frame:")
rot = T_camera_gripper[:3, :3]
print(f"  Camera X: {np.round(rot[:, 0], 4).tolist()}")
print(f"  Camera Y: {np.round(rot[:, 1], 4).tolist()}")
print(f"  Camera Z: {np.round(rot[:, 2], 4).tolist()}")

# Compute inverse: T_gripper_camera (gripper pose in camera frame)
T_gripper_camera = np.linalg.inv(T_camera_gripper)
print("\n" + "=" * 60)
print("Inverse: T_gripper_camera (gripper pose in camera frame)")
print("=" * 60)
print("  Rotation:")
print(T_gripper_camera[:3, :3])
print("  Translation:", T_gripper_camera[:3, 3])

# Test with actual data
print("\n" + "=" * 60)
print("Testing with actual coordinate transformation")
print("=" * 60)

# From test data
gripper_pos = np.array([0.1349, -0.0035, 0.2068])  # This is actually camera_pos already computed
# But let's use the end_pose that was published
gripper_pose_quat = np.array([0.698, 0.716, -0.012, 0.023])  # Approximate from earlier tests

# Simulate the camera_pose_publisher computation
cam_rot_gripper = R.from_quat(cam_quat).as_matrix()
cam_pos_world = gripper_pos  # This would be gripper_pos + gripper_rot @ cam_pos in actual code
cam_rot_world = cam_rot_gripper  # This would be gripper_rot @ cam_rot_gripper in actual code

print(f"\nIf we used camera_pose_publisher formula directly:")
print(f"  cam_pos_world = gripper_pos + gripper_rot @ cam_pos")
print(f"  But the end_pose published IS ALREADY camera pose!")

# Key insight
print("\n" + "=" * 60)
print("KEY FINDING")
print("=" * 60)
print("""
The camera_pose_publisher.py computes:
  cam_pos_world = gripper_pos + gripper_rot @ cam_pos_gripper
  cam_rot_world = gripper_rot @ cam_rot_gripper

This gives: T_camera_world = T_gripper_world @ T_camera_gripper

Then the forwarder publishes this as /piper/end_pose

On PGX, with --end-pose-is-camera-pose:
  world_pose = end_pose (camera_pose) @ optical_pose

This should be correct!

But wait - let's check if the hand-eye calibration
parameters might be defined in the wrong direction.
""")

# Try using inverse hand-eye
print("\nTesting with INVERSE hand-eye parameters:")
T_gripper_to_camera = T_camera_gripper  # What if this is actually T_gripper_to_camera?
T_camera_to_gripper = np.linalg.inv(T_gripper_to_camera)

# Simulate transformation with inverse
print("If hand-eye params should be inverted:")
print("  Camera position in gripper frame:", T_camera_to_gripper[:3, 3])
print("  Camera rotation in gripper frame:")
print(T_camera_to_gripper[:3, :3])

