#!/usr/bin/env python3
"""
Piper 机械臂解析逆运动学 (Python 实现)

参考: Agilex-College/piper/piper_kinematics/include/piper_analytical_ik.hpp
算法: 几何解耦法 - 先解臂部 (J1,J2,J3) 再解腕部 (J4,J5,J6)

用法:
    from piper_analytical_ik import PiperAnalyticalIK
    ik = PiperAnalyticalIK()
    solutions = ik.compute_ik(target_pose_4x4)
    best = ik.find_best_solution(target_pose_4x4, current_joints)
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


class PiperAnalyticalIK:
    """Piper 6-DOF 机械臂解析逆运动学

    改进 DH 参数:
      Joint 1: [α=0,      a=0,       d=0.123,   θ_offset=0]
      Joint 2: [α=-π/2,   a=0,       d=0,       θ_offset=-172.22°]
      Joint 3: [α=0,      a=0.28503, d=0,       θ_offset=-102.78°]
      Joint 4: [α=π/2,    a=-0.021984,d=0.25075, θ_offset=0]
      Joint 5: [α=-π/2,   a=0,       d=0,       θ_offset=0]
      Joint 6: [α=π/2,    a=0,       d=0.211,   θ_offset=0]

    关节限位:
      Joint 1: [-2.618, 2.618]  (~±150°)
      Joint 2: [0, π]           (只能向上抬)
      Joint 3: [-π, 0]          (只能向下弯)
      Joint 4: [-2.967, 2.967]  (~±170°)
      Joint 5: [-1.22, 1.22]    (~±70°)
      Joint 6: [-1.22, 1.22]    (~±70°)
    """

    # 改进 DH 参数 [alpha, a, d, theta_offset]
    DH_PARAMS = [
        [0.0,             0.0,      0.123,    0.0],
        [-np.pi / 2,     0.0,      0.0,      np.deg2rad(-172.22)],
        [0.0,             0.28503,  0.0,      np.deg2rad(-102.78)],
        [np.pi / 2,      -0.021984, 0.25075, 0.0],
        [-np.pi / 2,     0.0,      0.0,      0.0],
        [np.pi / 2,      0.0,      0.211,    0.0],
    ]

    JOINT_LIMITS = [
        [-2.618, 2.618],
        [0.0, np.pi],
        [-np.pi, 0.0],
        [-2.967, 2.967],
        [-1.22, 1.22],
        [-1.22, 1.22],
    ]

    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def __init__(self):
        self.d1 = self.DH_PARAMS[0][2]   # 0.123
        self.a2 = self.DH_PARAMS[2][1]   # 0.28503
        self.a3 = self.DH_PARAMS[3][1]   # -0.021984
        self.d4 = self.DH_PARAMS[3][2]   # 0.25075
        self.d6 = self.DH_PARAMS[5][2]   # 0.211

    @staticmethod
    def modified_dh_transform(alpha, a, d, theta):
        """计算改进 DH 变换矩阵"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct,            -st,            0,            a],
            [st * ca,       ct * ca,       -sa,          -sa * d],
            [st * sa,       ct * sa,        ca,           ca * d],
            [0,             0,              0,            1],
        ])

    def forward_kinematics(self, joint_values):
        """正运动学: 关节角度 -> 末端位姿 4x4 矩阵"""
        T = np.eye(4)
        for i in range(6):
            theta = joint_values[i] + self.DH_PARAMS[i][3]
            alpha = self.DH_PARAMS[i][0]
            a = self.DH_PARAMS[i][1]
            d = self.DH_PARAMS[i][2]
            T = T @ self.modified_dh_transform(alpha, a, d, theta)
        return T

    def compute_ik(self, target_pose, filter_by_limits=True):
        """计算所有可能的 IK 解

        Args:
            target_pose: 4x4 目标位姿矩阵 (末端执行器 link6 在 base_link 下的位姿)
            filter_by_limits: 是否过滤超出关节限位的解

        Returns:
            list of 6-element arrays, 每个 是一组关节角度
        """
        p_target = target_pose[:3, 3]
        R_target = target_pose[:3, :3]

        # Step 1: 计算腕部中心
        wrist_center = self._calculate_wrist_center(p_target, R_target)

        # Step 2: 解算臂部关节 J1, J2, J3
        arm_solutions = self._solve_arm_joints(wrist_center)
        if not arm_solutions:
            return []

        # Step 3: 对每个臂部解，解算腕部关节 J4, J5, J6
        all_solutions = []
        for arm_sol in arm_solutions:
            wrist_solutions = self._solve_wrist_joints(arm_sol, R_target)
            for wrist_sol in wrist_solutions:
                full_solution = list(arm_sol) + list(wrist_sol)

                # 应用关节偏移
                self._apply_joint_offsets(full_solution)

                # 归一化角度到 [-π, π]
                self._normalize_angles(full_solution)

                # 检查关节限位
                if not filter_by_limits or self._within_limits(full_solution):
                    all_solutions.append(np.array(full_solution))

        return all_solutions

    def find_best_solution(self, target_pose, current_joints, verbose=False):
        """找到最接近当前关节的 IK 解

        Args:
            target_pose: 4x4 目标位姿矩阵
            current_joints: 当前 6 关节角度 (list or array)
            verbose: 是否打印详细信息

        Returns:
            最佳解的 6 元素 numpy array, 无解则返回 None
        """
        solutions = self.compute_ik(target_pose, filter_by_limits=True)

        if not solutions:
            if verbose:
                print("[AnalyticalIK] No valid solutions found")
            return None

        if current_joints is None:
            return solutions[0]

        current = np.array(current_joints[:6])

        # 计算每个解到当前关节的距离
        best_solution = None
        min_distance = float('inf')

        for sol in solutions:
            dist = self._joint_distance(sol, current)
            if dist < min_distance:
                min_distance = dist
                best_solution = sol

        if verbose:
            fk_result = self.forward_kinematics(best_solution)
            pos_error = np.linalg.norm(fk_result[:3, 3] - target_pose[:3, 3])
            rot_error = np.linalg.norm(
                R.from_matrix(fk_result[:3, :3]).as_rotvec()
                - R.from_matrix(target_pose[:3, :3]).as_rotvec()
            )
            print(f"[AnalyticalIK] Best solution: "
                  f"joint_distance={min_distance:.4f}, "
                  f"pos_error={pos_error * 1000:.2f}mm, "
                  f"rot_error={np.degrees(rot_error):.2f}°")
            print(f"  Joints: {[f'{v:.3f}' for v in best_solution]}")

        return best_solution

    def compute_ik_with_verification(self, target_pose, current_joints=None,
                                      pos_tolerance=0.005, rot_tolerance_deg=5.0,
                                      verbose=False):
        """计算 IK 并验证解的正确性

        Args:
            target_pose: 4x4 目标位姿
            current_joints: 当前关节角度
            pos_tolerance: 位置误差容限 (m)
            rot_tolerance_deg: 旋转误差容限 (度)
            verbose: 是否打印详细信息

        Returns:
            (solution, verified) 或 (None, False)
        """
        solution = self.find_best_solution(target_pose, current_joints, verbose=verbose)
        if solution is None:
            return None, False

        # 用 FK 验证
        fk_result = self.forward_kinematics(solution)
        pos_error = np.linalg.norm(fk_result[:3, 3] - target_pose[:3, 3])

        try:
            rotvec_fk = R.from_matrix(fk_result[:3, :3]).as_rotvec()
            rotvec_target = R.from_matrix(target_pose[:3, :3]).as_rotvec()
            rot_error = np.linalg.norm(rotvec_fk - rotvec_target)
        except Exception:
            rot_error = float('inf')

        rot_error_deg = np.degrees(rot_error)
        verified = pos_error < pos_tolerance and rot_error_deg < rot_tolerance_deg

        if verbose or not verified:
            print(f"[AnalyticalIK] Verification: "
                  f"pos_error={pos_error * 1000:.2f}mm, "
                  f"rot_error={rot_error_deg:.2f}°, "
                  f"verified={verified}")

        return solution, verified

    def estimate_reachability(self, target_pose):
        """估算目标位姿的可达性 (不求解完整 IK)

        Returns:
            dict with reachability info
        """
        p_target = target_pose[:3, 3]
        R_target = target_pose[:3, :3]

        wrist_center = self._calculate_wrist_center(p_target, R_target)
        x, y, z = wrist_center
        z_rel = z - self.d1
        r = np.sqrt(x * x + y * y)

        L2 = self.a2
        L3 = np.sqrt(self.a3 ** 2 + self.d4 ** 2)

        D = (r * r + z_rel * z_rel - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)

        # 臂部可达性
        arm_reachable = abs(D) <= 1.0

        # 高度检查
        max_reach = L2 + L3
        dist_from_base = np.sqrt(r * r + z_rel * z_rel)
        distance_ok = dist_from_base <= max_reach * 1.1

        # approach 方向分析
        approach = target_pose[:3, 2]
        approach_z = float(approach[2])

        return {
            'arm_reachable': arm_reachable,
            'distance_ok': distance_ok,
            'D_value': float(D),
            'dist_from_base': float(dist_from_base),
            'max_reach': float(max_reach),
            'wrist_center': wrist_center.tolist(),
            'approach_z': approach_z,
            'likely_difficult': approach_z < -0.80 or not arm_reachable,
        }

    # --- Private methods ---

    def _calculate_wrist_center(self, p_target, R_target):
        """从目标位姿计算腕部中心位置"""
        z_6 = R_target[:, 2]
        return p_target - self.d6 * z_6

    def _solve_arm_joints(self, wrist_center):
        """解算臂部关节 J1, J2, J3"""
        x, y, z = wrist_center
        z_rel = z - self.d1  # 减去基座高度

        r = np.sqrt(x * x + y * y)

        L2 = self.a2   # 0.28503
        L3 = np.sqrt(self.a3 ** 2 + self.d4 ** 2)

        D = (r * r + z_rel * z_rel - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)

        if abs(D) > 1.0 + 1e-6:
            return []

        D = np.clip(D, -1.0, 1.0)

        beta = np.arccos(D)
        phi = np.arctan2(self.d4, abs(self.a3))  # a3 为负值

        solutions = []
        for sign in [1.0, -1.0]:
            q3 = sign * beta - phi
            k1 = L2 + L3 * np.cos(q3 + phi)
            k2 = L3 * np.sin(q3 + phi)
            gamma = np.arctan2(z_rel, r)
            delta = np.arctan2(k2, k1)
            q2 = gamma - delta
            q1 = np.arctan2(y, x)

            solutions.append([q1, q2, q3])

        return solutions

    def _solve_wrist_joints(self, arm_joints, R_target):
        """解算腕部关节 J4, J5, J6 使用 ZYZ 欧拉角"""
        q1, q2, q3 = arm_joints

        R03 = self._compute_R03(q1, q2, q3)
        R36 = R03.T @ R_target

        solutions = []
        r33 = R36[2, 2]

        if abs(r33) > 0.9999:  # 奇异位置
            q5 = 0.0 if r33 > 0 else np.pi
            q4 = 0.0
            q6 = np.arctan2(R36[1, 0], R36[0, 0])
            solutions.append([q4, q5, q6])
        else:
            for sign in [1.0, -1.0]:
                q5 = sign * np.arccos(np.clip(r33, -1.0, 1.0))
                q4 = np.arctan2(R36[1, 2], R36[0, 2])
                q6 = np.arctan2(R36[2, 1], -R36[2, 0])

                if sign < 0:
                    q4 += np.pi
                    q6 -= np.pi

                solutions.append([
                    np.arctan2(np.sin(q4), np.cos(q4)),
                    q5,
                    np.arctan2(np.sin(q6), np.cos(q6)),
                ])

        return solutions

    def _compute_R03(self, theta1, theta2, theta3):
        """计算从基座到关节 3 的旋转矩阵"""
        T = np.eye(4)
        for i, theta_raw in enumerate([theta1, theta2, theta3]):
            theta = theta_raw + self.DH_PARAMS[i][3]
            alpha = self.DH_PARAMS[i][0]
            a = self.DH_PARAMS[i][1]
            d = self.DH_PARAMS[i][2]
            T = T @ self.modified_dh_transform(alpha, a, d, theta)
        return T[:3, :3]

    def _apply_joint_offsets(self, joint_values):
        """应用关节偏移 (从原始角度减去 offset)"""
        for i in range(min(6, len(joint_values))):
            joint_values[i] -= self.DH_PARAMS[i][3]

    @staticmethod
    def _normalize_angles(joint_values):
        """归一化角度到 [-π, π]"""
        for i in range(len(joint_values)):
            joint_values[i] = (joint_values[i] + np.pi) % (2 * np.pi) - np.pi

    def _within_limits(self, joint_values):
        """检查关节角度是否在限位内"""
        for i in range(min(6, len(joint_values))):
            lo, hi = self.JOINT_LIMITS[i]
            if joint_values[i] < lo - 1e-6 or joint_values[i] > hi + 1e-6:
                return False
        return True

    @staticmethod
    def _joint_distance(joints1, joints2):
        """计算两组关节角度的距离 (考虑角度环绕)"""
        diff = np.array(joints1[:6]) - np.array(joints2[:6])
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        return float(np.linalg.norm(diff))


def test_analytical_ik():
    """测试解析 IK 的正确性"""
    ik = PiperAnalyticalIK()

    print("=== Piper Analytical IK Test ===\n")

    # Test 1: 已知关节角度 -> FK -> IK -> 验证
    test_configs = [
        [0.0, 0.5, -1.0, 0.0, 0.5, 0.0],
        [0.3, 0.8, -0.5, 0.2, -0.3, 0.1],
        [0.0, np.pi / 2, -np.pi / 2, 0.0, 0.0, 0.0],
        [0.5, 1.0, -1.5, 0.3, -0.5, 0.2],
    ]

    for idx, config in enumerate(test_configs):
        print(f"Test {idx + 1}:")
        print(f"  Input joints: {[f'{v:.4f}' for v in config]}")

        # FK
        target_pose = ik.forward_kinematics(config)
        print(f"  FK position: {np.round(target_pose[:3, 3], 5).tolist()}")

        # IK
        solutions = ik.compute_ik(target_pose, filter_by_limits=True)
        print(f"  IK solutions found: {len(solutions)}")

        if solutions:
            # 找最佳解
            best = ik.find_best_solution(target_pose, config)
            fk_check = ik.forward_kinematics(best)
            pos_error = np.linalg.norm(fk_check[:3, 3] - target_pose[:3, 3])
            print(f"  Best solution: {[f'{v:.4f}' for v in best]}")
            print(f"  Position error: {pos_error * 1000:.4f} mm")
            print(f"  Match: {'YES' if pos_error < 0.001 else 'NO'}")
        print()

    # Test 2: 可达性估算
    print("=== Reachability Test ===\n")

    test_poses = [
        ("前方桌面", np.array([0.4, 0.0, 0.2])),
        ("侧方低处", np.array([0.2, 0.3, 0.1])),
        ("正上方高处", np.array([0.0, 0.0, 0.6])),
        ("远处不可达", np.array([0.8, 0.0, 0.2])),
    ]

    for name, pos in test_poses:
        pose = np.eye(4)
        pose[:3, 3] = pos
        # 默认朝下
        pose[:3, :3] = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ], dtype=np.float64)

        reach_info = ik.estimate_reachability(pose)
        print(f"  {name} ({pos.tolist()}): "
              f"arm_reachable={reach_info['arm_reachable']}, "
              f"D={reach_info['D_value']:.4f}, "
              f"difficult={reach_info['likely_difficult']}")

    # Test 3: approach 方向分析
    print("\n=== Approach Direction Analysis ===\n")

    base_pos = np.array([0.4, 0.0, 0.2])
    for approach_z in [-0.95, -0.70, -0.50, -0.30]:
        pose = np.eye(4)
        pose[:3, 3] = base_pos
        approach = np.array([0.0, 0.0, approach_z])
        approach = approach / np.linalg.norm(approach)
        pose[:3, 2] = approach
        pose[:3, 0] = np.cross(np.array([0, 1, 0]), approach)
        pose[:3, 0] /= np.linalg.norm(pose[:3, 0])
        pose[:3, 1] = np.cross(approach, pose[:3, 0])

        solutions = ik.compute_ik(pose)
        reach_info = ik.estimate_reachability(pose)
        print(f"  approach_z={approach_z:.2f}: "
              f"solutions={len(solutions)}, "
              f"arm_reachable={reach_info['arm_reachable']}, "
              f"D={reach_info['D_value']:.4f}")


if __name__ == '__main__':
    test_analytical_ik()
