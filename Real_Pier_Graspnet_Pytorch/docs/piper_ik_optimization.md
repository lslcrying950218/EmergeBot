# Piper IK 优化方案

## 问题诊断

### IK 失败原因分析

之前的日志显示:
- approach Z 分量: -0.958 (接近垂直向下)
- 目标位置: 接近机械臂可达范围边缘

结合 Piper 关节限位:
| Joint | 范围 | 约束 |
|-------|------|------|
| 2 | [0, π] | 只能向上抬 |
| 3 | [-π, 0] | 只能向下弯 |

**垂直下抓姿态**需要:
1. Joint 2 接近 0 或 π（极端位置）
2. Joint 3 接近 0 或 -π（极端位置）
3. 这种姿态下的肘部几何可能无法到达目标

### DH 几何分析

Piper 机械臂结构（改进 DH）:
- d1 = 0.123m (基座高度)
- a2 = 0.28503m (大臂长度)
- a3 = -0.021984m (小臂偏移)
- d4 = 0.25075m (腕部高度)
- d6 = 0.211m (末端到 link6)

**可达高度范围估算**:
- 最低: d1 - a2 - d4 - d6 ≈ -0.42m (无法到达地面)
- 最高: d1 + a2 + d4 + d6 ≈ 0.77m

实际可达范围受关节限位约束，需要更精确的计算。

## 优化方案

### 方案 A: 改进 IK 请求策略（短期）

**1. 增加姿态变体数量**

当前 `_generate_ik_pose_variants()` 生成姿态变体，但数量有限。
建议增加更多变体:
- 绕 approach 轴旋转 ±15°, ±30°, ±45°
- 绕 grasp 轴旋转 ±45° (改变抓取方向)
- 绕 lateral 轴旋转 ±30°

**2. 改进种子策略**

```python
def _generate_ik_seeds(self, attempts):
    seeds = []
    # 1. 当前关节状态作为首选
    if self._current_joints:
        seeds.append([self._current_joints.get(n, 0.0) for n in self._arm_joints])
    # 2. 根据目标位置生成几何种子
    target_pos = pose_matrix[:3, 3]
    geometric_seed = self._estimate_seed_from_position(target_pos)
    seeds.append(geometric_seed)
    # 3. 随机种子覆盖关节限位边界
    for _ in range(attempts - 2):
        seeds.append(self._random_arm_seed())
    return seeds

def _estimate_seed_from_position(self, target_pos):
    # 根据目标位置估算合理的初始关节配置
    # 例如: 高位置 → joint2 较大, 低位置 → joint2 较小
    height = target_pos[2]  # z 坐标
    # 简化的几何估算
    j1 = np.arctan2(target_pos[1], target_pos[0])
    j2 = np.clip(np.arccos(height / 0.5), 0, np.pi)  # 简化估算
    j3 = -np.pi/2  # 默认肘部弯曲
    return [j1, j2, j3, 0, 0, 0]
```

**3. 添加 IK 成功统计和自适应**

```python
class GraspExecutor:
    def __init__(self):
        self._ik_success_stats = {
            'approach_z_bins': {},  # 按 approach z 分量统计成功率
        }
    
    def _get_approach_success_rate(self, approach_z):
        bin_key = round(approach_z, 1)  # -1.0, -0.9, -0.8, ...
        return self._ik_success_stats['approach_z_bins'].get(bin_key, 0.5)
```

### 方案 B: 集成解析 IK（中期）

**Python 实现解析 IK**

参考 `piper_analytical_ik.hpp` 的算法:

```python
class PiperAnalyticalIK:
    """Piper 机械臂解析逆运动学"""
    
    # DH 参数 (改进 DH)
    DH_PARAMS = {
        'a': [0, 0, 0.28503, -0.021984, 0, 0],
        'd': [0.123, 0, 0, 0.25075, 0, 0.211],
        'alpha': [0, -np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2],
        'theta_offset': [0, -172.22/180*np.pi, -102.78/180*np.pi, 0, 0, 0]
    }
    
    JOINT_LIMITS = [
        [-2.618, 2.618],   # J1
        [0, np.pi],        # J2
        [-np.pi, 0],       # J3
        [-2.967, 2.967],   # J4
        [-1.22, 1.22],     # J5
        [-1.22, 1.22],     # J6
    ]
    
    def compute_ik(self, target_pose, current_joints=None):
        """计算所有可能的 IK 解"""
        # 1. 计算腕部中心位置
        wrist_center = self._calculate_wrist_center(target_pose)
        
        # 2. 解算臂部关节 (J1, J2, J3)
        arm_solutions = self._solve_arm_joints(wrist_center)
        
        # 3. 对每个臂部解，解算腕部关节 (J4, J5, J6)
        all_solutions = []
        for arm_sol in arm_solutions:
            wrist_sols = self._solve_wrist_joints(arm_sol, target_pose)
            for wrist_sol in wrist_sols:
                full_sol = list(arm_sol) + list(wrist_sol)
                if self._within_limits(full_sol):
                    all_solutions.append(full_sol)
        
        # 4. 选择最接近当前关节的解
        if current_joints and all_solutions:
            return self._select_best_solution(all_solutions, current_joints)
        return all_solutions
    
    def _calculate_wrist_center(self, pose):
        """从目标位姿计算腕部中心"""
        d6 = self.DH_PARAMS['d'][5]
        z_axis = pose[:3, 2]  # approach 方向
        return pose[:3, 3] - d6 * z_axis
    
    def _solve_arm_joints(self, wrist_center):
        """解算 J1, J2, J3"""
        x, y, z = wrist_center
        z -= self.DH_PARAMS['d'][0]  # 减去基座高度
        
        r = np.sqrt(x*x + y*y)
        L2 = self.DH_PARAMS['a'][2]  # 0.28503
        L3 = np.sqrt(self.DH_PARAMS['a'][3]**2 + self.DH_PARAMS['d'][3]**2)
        
        D = (r*r + z*z - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        if abs(D) > 1.0:
            return []  # 无法到达
        
        solutions = []
        beta = np.arccos(np.clip(D, -1.0, 1.0))
        phi = np.arctan2(self.DH_PARAMS['d'][3], abs(self.DH_PARAMS['a'][3]))
        
        for sign in [1.0, -1.0]:
            q3 = sign * beta - phi
            k1 = L2 + L3 * np.cos(q3 + phi)
            k2 = L3 * np.sin(q3 + phi)
            gamma = np.arctan2(z, r)
            delta = np.arctan2(k2, k1)
            q2 = gamma - delta
            q1 = np.arctan2(y, x)
            
            solutions.append([q1, q2, q3])
        
        return solutions
    
    def _solve_wrist_joints(self, arm_joints, target_pose):
        """解算 J4, J5, J6 使用 ZYZ 欧拉角"""
        q1, q2, q3 = arm_joints
        R03 = self._compute_R03(q1, q2, q3)
        R36 = R03.T @ target_pose[:3, :3]
        
        solutions = []
        r33 = R36[2, 2]
        
        if abs(r33) > 0.9999:  # 奇异位置
            q5 = 0.0 if r33 > 0 else np.pi
            q4 = 0.0
            q6 = np.arctan2(R36[1, 0], R36[0, 0])
            solutions.append([q4, q5, q6])
        else:
            for sign in [1.0, -1.0]:
                q5 = sign * np.arccos(r33)
                q4 = np.arctan2(R36[1, 2], R36[0, 2])
                q6 = np.arctan2(R36[2, 1], -R36[2, 0])
                
                if sign < 0:
                    q4 += np.pi
                    q6 -= np.pi
                
                solutions.append([
                    np.arctan2(np.sin(q4), np.cos(q4)),
                    q5,
                    np.arctan2(np.sin(q6), np.cos(q6))
                ])
        
        return solutions
```

**集成到现有流程**

```python
def _solve_ik(self, pose_matrix, attempts=None):
    """使用解析 IK 优先，MoveIt 作为后备"""
    
    # 首先尝试解析 IK
    analytical_ik = PiperAnalyticalIK()
    solutions = analytical_ik.compute_ik(pose_matrix, self._current_joints)
    
    if solutions:
        self.get_logger().info(f"[ANALYTICAL IK] Found {len(solutions)} solutions")
        # 选择最佳解并验证
        best_solution = solutions[0] if len(solutions) == 1 else \
            analytical_ik.select_best_solution(solutions, self._current_joints)
        
        # 验证解的正确性
        if self._verify_ik_solution(best_solution, pose_matrix):
            return self._build_trajectory(best_solution)
    
    # 解析 IK 失败，使用 MoveIt 后备
    self.get_logger().warn("[ANALYTICAL IK] No solution, falling back to MoveIt")
    return self._solve_ik_moveit(pose_matrix, attempts)
```

### 方案 C: 改进抓取选择策略（长期）

**预过滤不可达抓取**

根据机械臂几何约束，在 IK 请求前过滤:

```python
def _filter_unreachable_grasps(self, grasps, scores, approach_vectors):
    """基于机械臂可达性预过滤抓取"""
    filtered_grasps = []
    
    for i, (grasp, score, approach) in enumerate(zip(grasps, scores, approach_vectors)):
        approach_z = approach[2]  # approach 方向的 Z 分量
        
        # 1. 检查高度可达性
        height = grasp[:3, 3][2]
        if height < 0.05 or height > 0.70:  # 超出可达范围
            continue
        
        # 2. 检查 approach 方向可达性
        # 根据历史统计，某些 approach 方向成功率极低
        success_rate = self._get_approach_success_rate(approach_z)
        if success_rate < 0.1:  # 过滤历史成功率 < 10% 的姿态
            continue
        
        # 3. 检查肘部空间约束
        # 估算肘部位置是否在物理可达范围内
        elbow_estimate = self._estimate_elbow_position(grasp)
        if not self._elbow_feasible(elbow_estimate):
            continue
        
        filtered_grasps.append(i)
    
    return filtered_grasps
```

**优先选择有利姿态**

```python
def _score_grasp_reachability(self, grasp, approach):
    """基于可达性对抓取评分"""
    approach_z = approach[2]
    
    # 1. 接近水平抓取 (approach_z ~ -0.5) 更容易
    horizontal_score = 1.0 - abs(approach_z + 0.5)
    
    # 2. 避免极端高度
    height = grasp[:3, 3][2]
    height_score = 1.0 if 0.15 < height < 0.55 else 0.5
    
    # 3. 避免极端 approach 方向
    # 极端向下 (z < -0.85) 和极端向上 (z > 0) 都不利
    approach_score = 0.0 if approach_z < -0.85 else 1.0
    
    return 0.4 * horizontal_score + 0.3 * height_score + 0.3 * approach_score
```

## 实施建议

### 第一阶段 (立即可实施)
1. 增加 IK 姿态变体数量
2. 添加基于位置的几何种子估算
3. 记录 IK 成功/失败统计，识别低成功率姿态

### 第二阶段 (需要测试验证)
1. 实现 Python 解析 IK
2. 使用解析 IK 作为主要方法，MoveIt 作为后备
3. 对比两种方法的成功率

### 第三阶段 (长期优化)
1. 建立 IK 可达性数据库
2. 在抓取生成阶段预过滤不可达姿态
3. 优化 Contact-GraspNet 的 approach 方向分布

## 参考

- Agilex-College/piper: piper_kinematics/include/piper_analytical_ik.hpp
- Agilex-College/piper: piper_kinematics/src/piper_ik_node_use_yaik.cpp (YAIK 符号推导)
- 当前实现: grasp_executor_motion.py