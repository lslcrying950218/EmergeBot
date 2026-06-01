#!/usr/bin/env python3
"""Test different optical frame transformations to find the correct one."""
import numpy as np
from scipy.spatial.transform import Rotation as R

# Test data from latest run
optical = np.array([0.2721, -0.1570, 0.3639])
camera_pos = np.array([0.1349, -0.0035, 0.2068])
camera_rot = np.array([
    [-0.0575, -0.7905, 0.6098],
    [-0.9981, 0.0579, -0.0191],
    [-0.0202, -0.6098, -0.7923]
])
actual_target = np.array([0.422, -0.165, 0.056])

print("=" * 70)
print("Testing different optical frame transformations")
print("=" * 70)
print(f"\nInput optical coords: {np.round(optical, 4).tolist()}")
print(f"Camera position: {np.round(camera_pos, 4).tolist()}")
print(f"Actual target: {np.round(actual_target, 4).tolist()}")

# Define test transformations
transforms = {
    "No transform (identity)": np.eye(3),
    "180 deg around Z (X->-X, Y->-Y)": np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]),
    "180 deg around X (Y->-Y, Z->-Z)": np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
    "180 deg around Y (X->-X, Z->-Z)": np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    "90 deg around Z (X->Y, Y->-X)": np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    "-90 deg around Z (X->-Y, Y->X)": np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]),
    "Swap X and Y": np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]]),
    "Invert Y only": np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]),
    "Invert X only": np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    "RealSense optical->camera (ROS std)": np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]),
}

results = []
for name, R_optical_to_cam in transforms.items():
    optical_in_cam = R_optical_to_cam @ optical
    camera_rot_corrected = camera_rot @ R_optical_to_cam
    world_pos = camera_pos + camera_rot_corrected @ optical
    error = world_pos - actual_target
    error_norm = np.linalg.norm(error)
    results.append((name, optical_in_cam, world_pos, error, error_norm))

# Sort by error norm
results.sort(key=lambda x: x[4])

print("\n" + "=" * 70)
print("Results sorted by error (smallest first)")
print("=" * 70)
for i, (name, optical_in_cam, world_pos, error, error_norm) in enumerate(results[:5], 1):
    print(f"\n{i}. {name}")
    print(f"   Optical in camera: {np.round(optical_in_cam, 4).tolist()}")
    print(f"   World position: {np.round(world_pos, 4).tolist()}")
    print(f"   Error: {np.round(error, 4).tolist()} (norm: {error_norm:.4f}m)")

print("\n" + "=" * 70)
print("Analysis of current error")
print("=" * 70)

# Current error breakdown
world_current = camera_pos + camera_rot @ optical
error_current = world_current - actual_target
print(f"\nCurrent calculation (no transform):")
print(f"  World: {np.round(world_current, 4).tolist()}")
print(f"  Actual: {np.round(actual_target, 4).tolist()}")
print(f"  Error: {np.round(error_current, 4).tolist()}")

# Break down by axis contribution
contrib_x = optical[0] * camera_rot[:, 0]
contrib_y = optical[1] * camera_rot[:, 1]
contrib_z = optical[2] * camera_rot[:, 2]

print(f"\nAxis contributions from optical coordinates:")
print(f"  optical_x (0.272): {np.round(contrib_x, 4).tolist()}")
print(f"  optical_y (-0.157): {np.round(contrib_y, 4).tolist()}")
print(f"  optical_z (0.364): {np.round(contrib_z, 4).tolist()}")

print(f"\nCamera axes in world frame:")
print(f"  Camera X axis (right): {np.round(camera_rot[:, 0], 4).tolist()}")
print(f"  Camera Y axis (down): {np.round(camera_rot[:, 1], 4).tolist()}")
print(f"  Camera Z axis (forward): {np.round(camera_rot[:, 2], 4).tolist()}")

# Key insight
print("\n" + "=" * 70)
print("Key observations")
print("=" * 70)
print("""
The Y-direction error (-0.126m) is significant. This suggests either:
1. The optical-to-camera frame transform is incorrect
2. The hand-eye calibration has an error
3. There's a systematic bias in the detection

Looking at the camera Y axis: [-0.7905, 0.0579, -0.6098]
This points in -X, +Y, -Z world direction (left-forward-down)

The optical Y coordinate is -0.157 (target is above camera center).
The contribution to world position is:
  -0.157 * [-0.7905, 0.0579, -0.6098] = [0.124, -0.009, 0.096]

This adds +X (right), -Y (back), +Z (up) to world position.
If the target is above center, and camera Y points down, this seems correct.

But the error is in -Y direction (偏右), suggesting either:
- The optical Y coordinate is wrong
- The camera Y axis direction is wrong
- There's a sign error somewhere
""")
