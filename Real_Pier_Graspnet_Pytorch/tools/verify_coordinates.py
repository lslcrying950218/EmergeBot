#!/usr/bin/env python3
"""
验证坐标转换的正确性

手眼标定公式 (Jetson端):
  T_camera_world = T_gripper_world @ T_camera_gripper
  即: camera_pose = gripper_pose @ hand_eye_transform

PGX端坐标转换 (当 end_pose_is_camera_pose=True):
  target_world = camera_pose @ target_optical
  即: target_world = R_camera @ target_optical + t_camera

验证方法：
  如果检测的是同一个目标，目标在世界坐标系中的位置应该是固定的。
  我们可以：
  1. 从 Frame 4 数据逆向求解相机旋转矩阵
  2. 验证该旋转是否合理（相机朝前）
  3. 计算目标的固定 world 坐标
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

print("=" * 60)
print("坐标转换验证")
print("=" * 60)

# 已知数据
# 1. 手动摆到目标位置时，相机的世界坐标（gripper位姿 + 手眼标定后）
camera_at_target_pos = np.array([0.384, 0.078, 0.138])

# 2. 测试时的数据 (Frame 4)
test_camera_pos = np.array([0.0642, 0.002, 0.1834])
test_grasp_optical = np.array([-0.0272, -0.0384, 0.2343])
test_grasp_world = np.array([0.298, -0.029, 0.144])

# 3. 手眼标定参数
hand_eye_translation = np.array([-0.0763, 0.0039, 0.035])
hand_eye_quat_xyzw = np.array([-0.120, 0.124, -0.697, 0.696])

# ============================================================================
# 步骤1：验证 Frame 4 的坐标转换公式
# ============================================================================
print("\n【步骤1：验证 Frame 4 坐标转换】")
print(f"相机位置 (world): {test_camera_pos}")
print(f"目标光学坐标: {test_grasp_optical}")
print(f"计算的世界坐标: {test_grasp_world}")

# 公式：world = R @ optical + t
# 所以：R @ optical = world - t
R_times_optical = test_grasp_world - test_camera_pos
print(f"\nR @ optical = world - t = {R_times_optical}")

# ============================================================================
# 步骤2：逆向求解相机旋转矩阵的主要特征
# ============================================================================
print("\n【步骤2：分析相机旋转】")
print("假设光学坐标主要贡献来自Z分量（前方），分析相机朝向...")

# 如果光学坐标只有 Z 分量 [0, 0, z]
# R @ [0, 0, z] = z * R[:, 2] = z * (相机Z轴在世界坐标系中的方向)
#
# 从 Frame 4 数据：
# R @ [-0.0272, -0.0384, 0.2343] ≈ [0.2338, -0.031, -0.0394]
#
# 光学 Z = 0.2343 对应的世界偏移大约是 0.2338 (X方向)
# 这说明相机Z轴（前方）主要指向世界 +X 方向

# 更精确的分析
# R @ optical = [dx, dy, dz]
# R[:, 0] * ox + R[:, 1] * oy + R[:, 2] * oz = [dx, dy, dz]

# 如果我们假设相机朝前（Z轴指向 +X），那么 R[:, 2] ≈ [1, 0, 0]
# 让我们验证

print("\n如果相机Z轴(前方)指向世界+X方向:")
print("  R[:, 2] ≈ [1, 0, 0]")
print("  那么光学Z=0.2343 应该产生世界X偏移约0.2343")

expected_R_col2 = np.array([1.0, 0.0, 0.0])  # 相机Z轴指向世界+X
expected_contribution_from_z = 0.2343 * expected_R_col2
print(f"  预期贡献 (仅Z分量): {expected_contribution_from_z}")
print(f"  实际 R @ optical: {R_times_optical}")

# 差异分析
residual = R_times_optical - expected_contribution_from_z
print(f"  差异 (来自光学XY): {residual}")
print(f"  光学XY: [{test_grasp_optical[0]}, {test_grasp_optical[1]}]")

# ============================================================================
# 步骤3：计算目标的固定世界坐标
# ============================================================================
print("\n【步骤3：计算目标固定世界坐标】")
print("假设：手动摆到目标时，相机正对目标，目标在相机视野中心")
print(f"手动摆位时相机位置: {camera_at_target_pos}")

# 如果目标在相机正前方（光学坐标 [0, 0, depth]）
# 相机朝前，所以目标世界坐标 ≈ [camera_x + depth, camera_y, camera_z]

# 但我们不知道目标的深度。让我们从 Frame 4 推断
print("\n从 Frame 4 推断目标深度:")
print(f"  目标光学Z = {test_grasp_optical[2]:.3f} m (目标在相机前方)")
print(f"  如果相机朝前，目标世界X ≈ 相机X + 光学Z")
expected_target_x = test_camera_pos[0] + test_grasp_optical[2]
print(f"  预期目标世界X = {test_camera_pos[0]:.3f} + {test_grasp_optical[2]:.3f} = {expected_target_x:.3f}")
print(f"  实际计算的世界X = {test_grasp_world[0]:.3f}")
print(f"  差异 = {abs(expected_target_x - test_grasp_world[0]):.3f} m")

# ============================================================================
# 步骤4：关键验证 - 相机旋转矩阵的正确性
# ============================================================================
print("\n【步骤4：验证相机旋转矩阵】")

# 从 R @ optical = delta_world，我们可以求解 R
# 这是一个欠定问题，但我们可以验证合理性

# 关键检查：如果相机朝前（Z轴指向+X），那么光学坐标转换为：
# world_x ≈ camera_x + optical_z (主要贡献)
# world_y ≈ camera_y + optical_x (相机X轴指向-Y?)
# world_z ≈ camera_z - optical_y (相机Y轴指向-Z?)

# 验证 Frame 4
print("Frame 4 验证:")
print(f"  光学: [{test_grasp_optical[0]:.3f}, {test_grasp_optical[1]:.3f}, {test_grasp_optical[2]:.3f}]")
print(f"  相机: [{test_camera_pos[0]:.3f}, {test_camera_pos[1]:.3f}, {test_camera_pos[2]:.3f}]")
print(f"  世界: [{test_grasp_world[0]:.3f}, {test_grasp_world[1]:.3f}, {test_grasp_world[2]:.3f}]")

# 计算 delta
delta_x = test_grasp_world[0] - test_camera_pos[0]
delta_y = test_grasp_world[1] - test_camera_pos[1]
delta_z = test_grasp_world[2] - test_camera_pos[2]
print(f"\n  世界坐标相对相机的偏移: [{delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f}]")

# 分析每个光学轴的贡献
# 假设相机坐标系：
#   - Z轴(前方) → 世界 +X (朝前)
#   - X轴(右) → 世界 -Y (右变左)
#   - Y轴(下) → 世界 -Z (向下)
# 这是典型的 "相机朝前，略向下" 的配置

print("\n假设的相机到世界坐标转换:")
print("  optical_Z (前) → world +X")
print("  optical_X (右) → world -Y")
print("  optical_Y (下) → world -Z")

# 验证
pred_delta_x = test_grasp_optical[2]  # optical_z → world_x
pred_delta_y = -test_grasp_optical[0]  # optical_x → world -y
pred_delta_z = -test_grasp_optical[1]  # optical_y → world -z

print(f"\n  预测偏移: [{pred_delta_x:.3f}, {pred_delta_y:.3f}, {pred_delta_z:.3f}]")
print(f"  实际偏移: [{delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f}]")
print(f"  误差: [{abs(delta_x-pred_delta_x):.3f}, {abs(delta_y-pred_delta_y):.3f}, {abs(delta_z-pred_delta_z):.3f}]")

# ============================================================================
# 步骤5：计算目标真实世界坐标
# ============================================================================
print("\n【步骤5：目标真实世界坐标】")

# 如果上述假设正确，我们可以从手动摆位计算目标坐标
# 手动摆位时，目标应该在相机视野中心附近
# 假设光学坐标约为 [0, 0, 0.2] (正前方20cm)

print(f"手动摆位时相机位置: {camera_at_target_pos}")
print("假设目标在相机正前方约 0.2m:")

for depth in [0.15, 0.20, 0.25]:
    target_optical_manual = np.array([0, 0, depth])
    target_world_manual = camera_at_target_pos + np.array([depth, 0, 0])
    print(f"  目标世界坐标 ≈ {target_world_manual}")

print("\n对比 Frame 4 计算结果:")
print(f"  Frame 4 计算的世界坐标: {test_grasp_world}")
print(f"  这与预期差异较大，可能原因:")
print(f"    1. 目标光学坐标不在中心 (X={test_grasp_optical[0]:.3f}, Y={test_grasp_optical[1]:.3f})")
print(f"    2. 相机旋转矩阵与假设不完全匹配")
print(f"    3. 手眼标定参数有误差")

# ============================================================================
# 结论
# ============================================================================
print("\n" + "=" * 60)
print("结论")
print("=" * 60)
print("""
需要进一步验证：
1. 在手动摆位时运行检测，获取目标的光学坐标
2. 用同一公式计算目标世界坐标
3. 验证计算结果是否与手动摆位位置一致

建议测试方案：
- 保持机械臂在测试位置不变
- 手动测量目标在基座坐标系中的位置
- 对比PGX计算的世界坐标
""")
