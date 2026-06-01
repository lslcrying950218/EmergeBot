#!/usr/bin/env python3
import numpy as np

# Frame 4 data
camera_pos = np.array([0.0843, 0.0039, 0.2923])
actual_target = np.array([0.394, 0.201, 0.074])
optical_z_detected = 0.448

# 计算实际距离
actual_vector = actual_target - camera_pos
actual_distance = np.linalg.norm(actual_vector)

print("Depth Estimation Analysis")
print("=" * 50)
print(f"Camera position: {np.round(camera_pos, 3).tolist()}")
print(f"Actual target: {np.round(actual_target, 3).tolist()}")
print(f"Actual distance: {actual_distance:.3f}m")
print(f"Detected optical Z: {optical_z_detected:.3f}m")
print(f"Depth error: {optical_z_detected - actual_distance:.3f}m ({(optical_z_detected/actual_distance - 1)*100:.1f}%)")
