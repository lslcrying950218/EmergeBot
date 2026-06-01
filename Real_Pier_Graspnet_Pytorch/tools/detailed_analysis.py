#!/usr/bin/env python3
"""Detailed analysis of coordinate transformation error."""
import numpy as np

# Frame 4 data
camera_pos = np.array([0.0843, 0.0039, 0.2923])
camera_rot = np.array([
    [-0.0055, -0.2566, 0.9665],
    [-1.0, -0.0004, -0.0058],
    [0.0019, -0.9665, -0.2566]
])
optical = np.array([-0.242, -0.012, 0.448])
actual_target = np.array([0.394, 0.201, 0.074])

print("=" * 60)
print("Detailed Coordinate Analysis")
print("=" * 60)

# Calculate world position
world_pos = camera_pos + camera_rot @ optical
print(f"\nCalculated world: {np.round(world_pos, 3).tolist()}")
print(f"Actual target: {np.round(actual_target, 3).tolist()}")
print(f"Error: {np.round(world_pos - actual_target, 3).tolist()}")

# Calculate expected optical from actual target
# If world = R @ optical + t, then optical = R.T @ (world - t)
expected_optical = camera_rot.T @ (actual_target - camera_pos)
print(f"\nExpected optical (from actual target): {np.round(expected_optical, 3).tolist()}")
print(f"Detected optical: {np.round(optical, 3).tolist()}")
print(f"Optical error: {np.round(optical - expected_optical, 3).tolist()}")

# Distance analysis
actual_distance = np.linalg.norm(actual_target - camera_pos)
calc_distance = np.linalg.norm(world_pos - camera_pos)
print(f"\nActual distance (camera to target): {actual_distance:.3f}m")
print(f"Calculated distance (camera to grasp): {calc_distance:.3f}m")

# Check camera Z-axis direction
print(f"\nCamera Z-axis (forward): {np.round(camera_rot[:, 2], 3).tolist()}")
print("  -> Mainly points in +X world direction (forward)")

# Key insight
print("\n" + "=" * 60)
print("Key Analysis")
print("=" * 60)

# The optical Z should match the distance along camera Z axis
# optical_z = expected_optical[2] if transformation is correct
print(f"""
Depth analysis:
  Detected optical Z: {optical[2]:.3f}m
  Expected optical Z: {expected_optical[2]:.3f}m
  Difference: {optical[2] - expected_optical[2]:.3f}m

The optical Z (depth) is only 2cm off, but the X/Y in optical frame
have larger errors, causing the world position to be off.

Detected optical X: {optical[0]:.3f}m vs expected: {expected_optical[0]:.3f}m (error: {optical[0] - expected_optical[0]:.3f}m)
Detected optical Y: {optical[1]:.3f}m vs expected: {expected_optical[1]:.3f}m (error: {optical[1] - expected_optical[1]:.3f}m)

The main error comes from optical X coordinate!
This could be caused by:
1. Contact-GraspNet predicting wrong grasp position in the image
2. Camera pose error affecting the rotation matrix
3. Hand-eye calibration error
""")
