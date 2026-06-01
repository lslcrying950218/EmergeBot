---
name: navigate-to-object
description: Navigate DimOS robot to static objects using natural language queries
version: 1.0.0
author: Hermes
tags: [dimos, navigation, object-detection, vlm]
---

# Navigate to Static Object

Navigate the DimOS robot to a static object specified by natural language query.

## Description

This skill enables the robot to navigate to static objects (e.g., "wall", "box", "bottle") using:
1. VLM (Vision Language Model) for object detection
2. CSRT tracker for 2D bounding box tracking
3. BBoxNavigationModule for goal point calculation
4. ReplanningAStarPlanner for path planning and navigation

## Prerequisites

### Blueprint
Use `unitree-go2-agentic` blueprint (已合并导航功能):
- `ObjectTracker2D` - CSRT-based 2D object tracker
- `BBoxNavigationModule` - Converts bbox to navigation goals
- `NavigationSkillContainer` - Contains `navigate_to_object` skill

**注意：** 原 `unitree-go2-agentic-nav` 蓝图已废弃，导航功能已合并到主蓝图。

### Dependencies
- OpenCV with contrib (for CSRT tracker)
- VLM model configured for object detection
- Camera intrinsics published on `/camera_info`

## Usage

### Start DimOS with Navigation Blueprint

**速度控制（重要）：** 机器狗移动过快可能导致检测到目标后立即丢失。使用 `DIMOS_NERF_SPEED` 环境变量降低速度：
- 默认速度：0.55 m/s
- 推荐速度：`DIMOS_NERF_SPEED=0.1` → 0.055 m/s（极慢，便于稳定检测）

```bash
# 推荐启动命令（仿真，低速）
DIMOS_NERF_SPEED=0.1 dimos --simulation --viewer none run unitree-go2-agentic --daemon
```

**速度说明：**
- `DIMOS_NERF_SPEED` 同时控制探索和导航速度
- 探索流程：WavefrontFrontierExplorer 发布目标 → ReplanningAStarPlanner 使用 LocalPlanner 导航
- `LocalPlanner._speed = 0.55 * nerf_speed`
- DirectCmdVelExplorer（linear_speed=0.8）是旧代码，未被使用

### Call navigate_to_object
```bash
dimos mcp call navigate_to_object --json-args '{"query": "wall", "goal_distance": 1.0}'
```

### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| query | string | required | Natural language description of the object |
| goal_distance | float | 0.5 | Distance to stop from the object (meters). Default changed from 0.3 to 0.5 for better direction estimation during approach. |

### Return Values
- `"Successfully arrived near '{query}'"` - Navigation completed
- `"Object '{query}' not found in camera view"` - VLM couldn't detect the object
- `"Tracking lost - could not navigate to '{query}'"` - Object lost before navigation started
- `"Navigation to '{query}' timed out after X seconds"` - Timeout reached

## Technical Details

### Architecture Flow
```
navigate_to_object()
    ↓
VLM Detection (visual.query_single_bbox)
    ↓
ObjectTracker2D.track(bbox)
    ↓
CSRT Tracker updates at ~5Hz
    ↓
Detection2DArray published
    ↓
BBoxNavigationModule converts to PoseStamped
    ↓
ReplanningAStarPlanner navigates
```

### Key Files
- `dimos/agents/skills/navigation.py` - navigate_to_object skill method
- `dimos/perception/object_tracker_2d.py` - CSRT-based 2D tracker
- `dimos/navigation/bbox_navigation.py` - BBox to 3D goal conversion (**已添加点云深度估计**)
- `dimos/perception/detection/type/detection3d/pointcloud.py` - Detection3DPC.from_2d() 点云投影
- `dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py` - Main blueprint

### 坐标系变换（关键修复 2026-05-15）

**问题现象：** 无论机器人在哪里启动，检测到目标后都会导航到固定位置 `(0.30, -0.xx, 0.xx)`（原点附近）。

**根本原因（双重 bug）：**

1. **Bug 1: TF 变换失败 + 危险 fallback**
   - 实机 odom 消息没有设置 `frame_id`，TF 树缺少 `odom → base_link` 根节点
   - `self.tf.get("odom", "camera_link")` 返回 None
   - fallback 逻辑错误地将相机坐标系坐标当作全局坐标：
     ```python
     # bbox_navigation.py 第168-174行（旧代码）
     if goal_world is None:
         goal_world = goal_cam  # 错误！相机相对坐标被当全局坐标
     ```
   
2. **Bug 2: GlobalPlanner 忽略 frame_id**
   - `handle_goal_request()` 直接使用 `goal.position`，完全忽略 `goal.frame_id`
   - 目标 `(0.30, ...)` 被当作全局坐标执行

**修复（两个文件）：**

1. **connection.py** - 强制设置 frame_id
2. **bbox_navigation.py** - 移除危险 fallback

**TF 树结构（完整链）：**
```
world → odom → base_link → camera_link → camera_optical
```

### 点云深度估计 TF 链修复（2026-05-15）

**问题：** `Detection3DPC.from_2d()` 需要 `world → camera_optical` 变换，但 GO2Connection 只发布 `odom → base_link`。

**GO2 的 frame_id 分布：**
- 点云 `frame_id = "world"`（硬编码在 `lidar.py`）
- 里程计 `frame_id = "odom"`
- 图像 `frame_id = "camera_optical"`
- GO2Connection 只发布 `odom → base_link`

**缺失的 TF 变换：**
1. `world → odom`：单位变换（都是固定世界坐标系）
2. `base_link → camera_link`：相机在机器人前方 30cm
3. `camera_link → camera_optical`：坐标系旋转

**修复（bbox_navigation.py start() 方法）：**
```python
# world -> odom: Identity transform
world_to_odom_tf = Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="world",
    child_frame_id="odom",
)

# base_link -> camera_link: camera mounted 30cm forward
camera_link_tf = Transform(
    translation=Vector3(0.3, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
)

# camera_link -> camera_optical: coordinate system rotation
# camera_link (X=forward, Y=left, Z=up) -> camera_optical (X=right, Y=down, Z=forward)
camera_optical_tf = Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
)

# Publish periodically
self._disposables.add(
    rx.interval(1.0).subscribe(
        lambda _: self.tf.publish(world_to_odom_tf, camera_link_tf, camera_optical_tf)
    )
)
```

**日志验证：**
- ✓ 正确：`Pointcloud depth estimation: N points, centroid_world=(...), depth=X.XXm`
- ✗ 错误：`Could not get transform from world to camera_optical`

### 相机内参分辨率缩放（关键修复 2026-05-15）

**问题：** 图像可能是缩放后的（如 320x240），但 camera_info 内参是全分辨率的（如 1280x720）。直接使用会导致投影错误。

**修复（_scaled_camera_info_for_image 方法）：**
```python
def _scaled_camera_info_for_image(self, image: Image):
    """Scale camera intrinsics to match actual image resolution."""
    width = image.width
    height = image.height
    scale_x = width / self._camera_info.width
    scale_y = height / self._camera_info.height

    K = list(self._camera_info.K)
    if len(K) == 9:
        K[0] *= scale_x  # fx
        K[2] *= scale_x  # cx
        K[4] *= scale_y  # fy
        K[5] *= scale_y  # cy

    lcm_camera_info.K = K
    lcm_camera_info.width = width
    lcm_camera_info.height = height
    return lcm_camera_info
```

**日志验证：**
- ✓ 正确：`Scaling camera intrinsics: 1280x720 -> 320x240, scale=(0.25, 0.33)`

### 点云深度估计完整流程（已修复）

```python
def _estimate_depth_from_pointcloud(self, det: Detection2DArray):
    # 1. 创建 Detection2DBBox（使用工厂方法）
    detection_2d = Detection2DBBox.from_ros_detection2d(
        ros_det, image=self._latest_image, name=f"class_{class_id}"
    )

    # 2. 获取 TF 变换（world -> camera_optical）
    world_to_optical = self.tf.get(pointcloud_frame, "camera_optical")

    # 3. 缩放相机内参
    lcm_camera_info = self._scaled_camera_info_for_image(self._latest_image)

    # 4. 点云投影获取 3D 检测
    detection_3d = Detection3DPC.from_2d(
        det=detection_2d,
        world_pointcloud=self._latest_pointcloud,
        camera_info=lcm_camera_info,
        world_to_optical_transform=world_to_optical,
        filters=[],
    )

    # 5. 过滤地面点 (z > 0.3m in world frame)
    points, _ = detection_3d.pointcloud.as_numpy()
    height_mask = points[:, 2] > 0.3
    filtered_points = points[height_mask]

    # 6. 计算质心并变换到 camera_optical 坐标系
    centroid_world = filtered_points.mean(axis=0)
    centroid_homogeneous = np.array([centroid_world[0], centroid_world[1], centroid_world[2], 1.0])
    centroid_optical = (world_to_optical.to_matrix() @ centroid_homogeneous)[:3]

    # 7. 提取深度（Z in optical frame = forward distance）
    depth = centroid_optical[2]

    # 8. 转换到 camera_link 坐标系
    # optical (X=right, Y=down, Z=forward) -> camera_link (X=forward, Y=left, Z=up)
    return (centroid_optical[2], -centroid_optical[0], -centroid_optical[1])
```

**关键点：**
- 使用 `Detection2DBBox.from_ros_detection2d()` 避免手动构造错误
- 使用矩阵乘法 `to_matrix() @ point` 变换点，**不要用** `Transform.apply()`
- 光学坐标系 Z 轴 = 深度（向前方向的距离）

### Verification Results (2026-05-14)

**测试环境：** 仿真模式，`DIMOS_NERF_SPEED=0.1`

**测试结果：**
- ✓ 探索在 `navigate_to_object` 调用时正确停止（EXPLORATION_STOPPED 日志出现）
- ✓ VLM 成功检测静态物体（鞋子：bbox 46.08, 173.76, 78.08, 192.96）
- ✓ BBoxNavigationModule 自动生成导航目标
- ✓ 导航耗时 4.497 秒到达目标
- ✓ 到达后机器人保持静止，无继续移动问题

**关键日志证据（测试目录：20260514-161802-unitree-go2-agentic）：**
```
08:18:52 - EXPLORATION_STARTED
08:19:08.509 - navigate_to_object called
08:19:08.946 - EXPLORATION_STOPPED (0.437s 响应)
08:20:37 - Found 鞋子 at bbox: (46.08, 173.76, 78.08, 192.96)
08:20:40 - Navigation state=IDLE, goal_reached=True
```

## Known Limitations
1. ~~**深度估计错误（关键问题，待修复）**~~ → **已修复（2026-05-15）** - 现使用激光雷达点云 + `Detection3DPC.from_2d()` 获取真实深度，回退到固定深度时会在日志中标记 `[depth=fallback]`
2. **CSRT Tracker**: Sensitive to lighting changes and occlusion
3. **Static Objects Only**: Not designed for moving targets (use `look_out_for` for people)
4. **Path Planning**: May fail if target is unreachable or blocked by obstacles
5. **Small Objects**: Small bboxes may cause unstable goal points
6. **VLM Color Hallucination**: VLM may misidentify colors (e.g., seeing "green" when object is dark/black), especially when colored objects are nearby

## Race Condition Fixes (2026-05-14)

### 问题1：探索停止后机器人仍移动到错误位置
**症状：** 检测到目标后，`_handle_match` 停止探索并取消导航，但机器人仍继续执行上一个探索目标路径

**根因：**
- `_handle_match` 中 `cancel_goal()` 是异步的，探索模块关闭时可能发布延迟目标
- 没有 `Twist.zero()` 命令让机器人立即停止（person follow 有，navigate_to_object 没有 `cmd_vel` 接口）

**修复（navigation.py）：** 模仿 person follow 的 `_pause_motion_for_tracking_handoff`：
```python
# navigate_to_object 中，停止探索/巡逻后：
self._navigation.cancel_goal()   # 第一次取消：立即清除当前目标
time.sleep(0.5)                  # 等待机器人稳定
self._navigation.cancel_goal()   # 第二次取消：清除延迟到达的目标
```

**修复（wavefront_frontier_goal_selector.py）：** `stop_exploration()` 不应发布导航目标：
```python
# 旧代码（已删除）：发布当前位置作为目标，导致竞争
# 新代码：不发布任何目标，由调用者负责 cancel_goal()
```

### 问题2：BBoxNavigationModule 目标更新过快
**症状：** 检测到目标，机器人原地晃动，状态循环 `initial_rotation → idle → initial_rotation`

**根因：** ObjectTracker2D 以 ~5Hz 发布检测，BBoxNavigationModule 每次都发布新 goal_request，导致 GlobalPlanner 不断取消重规划

**修复（bbox_navigation.py）：** 添加 `min_goal_interval` 参数（类似 person follow 的 `_goal_update_interval=0.5`）：
```python
class Config(ModuleConfig):
    min_goal_interval: float = 0.5  # 最小目标更新间隔（秒）

# _on_detection 中添加时间检查
current_time = time.time()
if self._last_goal_time > 0 and current_time - self._last_goal_time < self.config.min_goal_interval:
    return  # 跳过过快的更新
```

### 与 Person Follow 的功能对齐（已完成）

| 功能 | person follow | navigate_to_object |
|------|---------------|-------------------|
| 停止探索/巡逻 | ✅ | ✅ |
| 第一次 cancel_goal | ✅ | ✅ |
| 发送 Twist.zero() | ✅ | ✅（已添加 cmd_vel 接口） |
| 等待稳定 | ✅ | ✅（0.5s） |
| 第二次 cancel_goal | ✅ | ✅ |
| min_goal_interval | ✅（0.5s） | ✅（0.5s） |
| 调整视角居中 | ✅ | ✅（2026-05-15 新增） |

### 简化的 navigate_to_object 流程（2026-05-15，最新更新）

**设计原则：** 每个中间步骤都可能失败，累积故障率很高。简化流程，减少失败点。

**已移除的中间步骤：**
- ~~等待稳定 0.5s~~ - 增加 latency，目标可能移出视野
- ~~重新检测 `_get_bbox_for_current_frame`~~ - VLM 再次检测可能失败
- ~~居中逻辑 `_center_target_before_navigation`~~ - 增加失败点，边缘目标易丢失
- ~~one_shot 模式~~ - 破坏逐步逼近设计，导航位置错误（**已确认移除**）

**当前流程：**
```
look_out_for 检测到目标 → 验证面积/边缘
    ↓
navigate_to_object:
    ├── cancel_goal() + Twist.zero()  # 停止探索和运动
    ├── 直接用 initial_bbox           # 不重新检测
    └── _object_tracking.track(bbox)  # 启动 CSRT 追踪
    ↓
ObjectTracker2D 每帧追踪 (~5Hz)
    ↓
BBoxNavigationModule:
    ├── 计算 bbox 中心
    ├── 用 goal_distance=0.5m 假设深度计算相机坐标
    ├── TF 变换: camera_link → odom
    ├── 检查位置变化 > 0.2m
    ├── 检查时间间隔 > 0.5s
    └── 发布新目标（逐步逼近）
    ↓
机器人逐步接近目标 → 目标变大 → 位置更准确 → 收敛
    ↓
到达目标附近 (~goal_distance)
    ↓
_center_target_after_navigation:  # 新增：导航后居中
    ├── Phase 1: 搜索（目标不在画面）
    │   ├── 小角度旋转 (~30° per step)
    │   ├── 左6步→右6步 覆盖360°
    │   └── 每步 VLM 检测
    └── Phase 2: 比例控制居中
        ├── 计算 offset = bbox_center - image_center
        ├── angular_z = -offset * gain (比例控制)
        ├── 短脉冲旋转 0.15s
        ├── VLM 重新检测
        └── 重复最多 15 次 (±8% margin)
```

**当前配置（perceive_loop_skill.py）：**
```python
_navigate_to_object_min_area_ratio: float = 0.0015  # 0.15% 面积阈值
_navigate_to_object_edge_margin_ratio: float = 0.03  # 3% 边缘检测
```

**当前配置（bbox_navigation.py）：**
```python
goal_distance: float = 0.5  # 折中值，方向估计更准确
position_threshold: float = 0.2  # 位置变化 > 0.2m 才更新
min_goal_interval: float = 0.5  # 发布间隔 > 0.5s
# one_shot 已移除 - 恢复逐步逼近设计
```

**注意：** 面积阈值 0.0015 (0.15%) 仍高于真实小目标（0.04%~0.10%），可能拒绝正确目标。如遇检测被拒绝，需降低到 **0.0003 (0.03%)** 或 **0.0004 (0.04%)**。

### 导航后居中（_center_target_after_navigation）

**目的：** 导航到达后，调整机器人位姿使目标处于画面中间。

**设计原则：** 参考 `person_follow` 的居中方式，使用小角度渐进式旋转配合实时检测，而非大角度跳跃旋转。

**参数：**
```python
margin_ratio: float = 0.15        # 15% margin（已放宽，更容易通过）
angular_gain: float = 0.9         # 比例控制增益
min_angular_speed: float = 0.15   # 最小角速度 (rad/s)
max_angular_speed: float = 0.5    # 最大角速度 (rad/s)
max_attempts: int = 15            # 居中迭代次数
search_angular_speed: float = 0.25 # 搜索阶段角速度 (rad/s)
```

**两阶段流程：**

**Phase 1 - 搜索阶段（目标不在画面时）：**
```
VLM 检测失败
    ↓
小角度旋转 (~30° per step)
    ↓
旋转参数: 0.25 rad/s × 1.0s ≈ 0.25 rad ≈ 14°
    ↓
策略: 左转6步(~180°) → 右转6步(~180°)
    ↓
每步后 VLM 检测
    ↓
找到目标 → 进入 Phase 2
找不到 → 返回 False
```

**Phase 2 - 居中阶段（目标已找到）：**
```
计算偏移量: offset_x = bbox_center - image_center
    ↓
比例控制计算角速度:
  normalized_offset = offset_x / (image_width / 2)
  angular_z = -normalized_offset * angular_gain
    ↓
限制角速度范围: [min_angular_speed, max_angular_speed]
    ↓
短时间旋转: angular_z × 0.15s
    ↓
VLM 重新检测
    ↓
检查是否居中 (±8% margin)
    ↓
重复最多 15 次
```

**关键区别（vs 旧方案）：**
| 特性 | 旧方案 | 新方案 |
|------|--------|--------|
| 搜索角度 | 90° 大跳跃 | 30° 小步渐进 |
| 居中控制 | 固定速度 | 比例控制（偏差大→转速快） |
| 旋转时长 | 0.3s 固定 | 0.15s 短脉冲 |
| 容差 | 20% | 8% |
| 迭代次数 | 5 次 | 15 次 |

**设计参考：** `person_follow.py` 的 `_compute_warmup_centering_twist()` 方法使用类似的比例控制策略。

### goal_distance 的真实含义（关键理解）

**核心概念：** `goal_distance` 是**假设的目标深度 Z**，用于 2D→3D 反投影，不是"最终距离目标的距离"。

**数学推导：**
```
相机投影模型（3D → 2D）:
  u = fx * X / Z + cx
  v = fy * Y / Z + cy

反投影（2D → 3D，需要假设 Z）:
  X = (u - cx) * Z / fx
  Y = (v - cy) * Z / fy
  Z = goal_distance（假设值）
```

**计算逻辑：**
```python
# bbox_navigation.py
x_cam = (center_x - cx) / fx * goal_distance  # X 坐标取决于假设 Z
y_cam = (center_y - cy) / fy * goal_distance  # Y 坐标取决于假设 Z
z_cam = goal_distance  # ← 固定假设，不是真实距离
```

**逐步逼近原理（one_shot=False 时）：**
```
机器人距目标 3m 远:
  计算: odom_x = robot_x + goal_distance = 0 + 0.3 = 0.3m
  导航到 0.3m → 发现目标还在前方 2.7m
  CSRT 继续追踪 → 重新计算位置

机器人距目标 1m 远:
  计算: odom_x = 2.0 + 0.3 = 2.3m
  导航到 2.3m → 发现目标还在前方 0.7m
  CSRT 继续追踪 → 重新计算位置

机器人距目标 0.3m 远:
  计算: odom_x = 2.7 + 0.3 = 3.0m = 目标真实位置 ✓
```

**结论：** 逐步逼近下，最终停在距目标约 `goal_distance` 处（收敛后）。

### goal_distance 大小的权衡

| goal_distance | 逼近过程 | 最终距离 | 方向估计 |
|---------------|----------|----------|----------|
| **0.3m** | 假设误差大，规划器可能困惑 | 停得近 | 偏差大 |
| **0.5m** | 折中 | 合适 | 折中 |
| **1.0m** | 假设误差小，逼近平滑 | 停得远 | 偏差小 |

**具体例子：** 目标在右侧 0.85m 处，距离 3m：
```
goal_distance=0.3m: X_calc=0.08m (真实0.85m, 方向偏差0.77m)
goal_distance=1.0m: X_calc=0.28m (真实0.85m, 方向偏差0.57m)
```

**建议：** 使用 **0.5m** 或 **1.0m**，方向估计更准确。

### one_shot 模式问题与修复（2026-05-15）

**问题：** one_shot=True 破坏了逐步逼近设计！

| 模式 | 目标漂移 | 距离准确 | 适用场景 |
|------|----------|----------|----------|
| one_shot=True | ✓ 无漂移 | ✗ 固定假设距离，**完全错误** | 目标已在 goal_distance 范围内 |
| one_shot=False | ✗ 可能漂移 | ✓ 逐步逼近，最终准确 | 目标距离未知或较远 |

**修复（方案A）：** 已移除 one_shot 模式，恢复逐步逼近设计。

**当前实现：**
```python
# bbox_navigation.py - 已删除以下内容
# one_shot: bool = True  # 已移除
# tracking_reset_timeout: float = 2.0  # 已移除
# _goal_published: bool = False  # 已移除
# reset_goal() 方法  # 已移除

# 保留正确的修复:
# - TF 变换: camera_link → odom
# - position_threshold: 位置变化 > 0.2m 才发布新目标
# - min_goal_interval: 发布间隔 > 0.5s
# - goal_distance = 0.5m（折中值）
```

### 原始版本 vs 修复版本对比

**原始版本（git 51355665a）：**
```python
# bbox_navigation.py - ~60 行
goal = PoseStamped(
    position=Vector3(z, -x, -y),
    frame_id=det.header.frame_id,  # ← camera_link，错误！
)
goal_request.publish(goal)
# 无 one_shot，每帧发布新目标
```

**当前版本：**
```python
# bbox_navigation.py - ~200 行
# TF 变换：camera_link → odom（正确）
goal_world = self._transform_point(goal_cam, "camera_link", "odom")
goal = PoseStamped(position=goal_world, frame_id="odom")
# 无 one_shot，恢复逐步逼近设计
# position_threshold: 位置变化 > 0.2m 才更新
# min_goal_interval: 时间间隔 > 0.5s 才更新
```

**关键修复历史：**
- ✓ TF 变换到 odom（导航系统需要全局坐标）
- ✓ one_shot 防止漂移（但破坏了逐步逼近，已移除）
- ✗ centering 等中间步骤（已简化移除）

**当前工作流程：**
```
检测 → 停止 → 追踪 → 逐步逼近 → 到达目标
```

**代码修改：**
1. `navigation.py` - 添加导入和流定义：
   ```python
   from dimos.msgs.geometry_msgs.Twist import Twist
   from dimos.core.stream import In, Out
   
   class NavigationSkillContainer(Module):
       cmd_vel: Out[Twist]  # 新增
   ```

2. `navigation.py` - `navigate_to_object` 方法添加 `Twist.zero()`：
   ```python
   # 第二次 cancel_goal 后
   self.cmd_vel.publish(Twist.zero())  # 立即停止机器人运动惯性
   ```

3. 蓝图自动连接：`autoconnect` 会自动将 `NavigationSkillContainer.cmd_vel` 连接到 `GO2Connection.cmd_vel`，无需手动配置。

## Design Lessons

### 充分调研现有代码再设计新方案（重要教训）
**教训（2026-05-15）：**
设计 `BBoxNavigationModule` 时，使用了固定深度假设 `goal_distance=0.5m`，导致导航距离错误。

但实际上 DimOS 已有完整的点云+检测导航方案：
- `DetectionNavigation` 类（`navigation/visual_servoing/detection_navigation.py`）
- `Detection3DPC.from_2d()` - 点云投影获取真实深度
- `_compute_robust_target_position()` - 稳健的目标位置计算

**正确做法：**
1. 设计新模块前，搜索关键词如 `detection_navigation`、`Detection3D`、`pointcloud.*bbox`、`depth.*bbox`
2. 检查是否有现成的解决方案可以复用或参考
3. 用户说"没有深度相机"时，要确认激光雷达点云是否可用（Go2 有 `lidar: Out[PointCloud2]`）
4. 检查 `dimos/perception/detection/type/detection3d/pointcloud.py` 等底层模块

### DimOS 消息类型结构（关键细节）

**Detection2D 结构（非标准 ROS2）**：
```python
# 错误 - Detection2D 没有这些属性
det.detections[0].name        # ✗ 不存在
det.detections[0].confidence  # ✗ 不存在

# 正确 - 使用 results 数组
det.detections[0].results[0].hypothesis.class_id  # ✓ 类别 ID
det.detections[0].results[0].hypothesis.score      # ✓ 置信度
det.detections[0].id                               # ✓ 追踪 ID（字符串）
det.detections[0].bbox.center.position.x          # ✓ bbox 中心
det.detections[0].bbox.size_x, .size_y            # ✓ bbox 尺寸
```

**Detection2DBBox 构造函数（必需所有参数）**：
```python
# 错误 - 缺少参数
Detection2DBBox(name="target", bbox=[x1,y1,x2,y2], confidence=1.0)  # ✗

# 正确 - 完整参数
Detection2DBBox(
    bbox=(x_min, y_min, x_max, y_max),  # 注意：元组，非列表
    track_id=0,
    class_id=-1,          # 整数，非字符串
    confidence=1.0,
    name="target",       # 字符串类别名
    ts=timestamp,        # float 秒
    image=image_obj,     # Image 对象
)
```

**时间戳处理（LCM vs ROS2）**：
```python
# 错误 - 属性名不同
ts.nanosec  # ✗ LCM 使用 nsec，ROS2 使用 nanosec

# 正确 - 使用工具函数
from dimos.types.timestamped import to_timestamp
ts = to_timestamp(header.stamp)  # 自动处理 nsec/nanosec
```

**Detection2DBBox 创建（推荐方式）**：
```python
# 错误 - 手动构造容易出错
Detection2DBBox(
    bbox=(x_min, y_min, x_max, y_max),
    track_id=...,
    class_id=...,
    confidence=...,
    name=...,
    ts=...,
    image=...,
)  # 参数多，易遗漏

# 正确 - 使用工厂方法
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
detection_2d = Detection2DBBox.from_ros_detection2d(
    ros_det,  # det.detections[0]
    image=self._latest_image,
    name=f"class_{ros_det.results[0].hypothesis.class_id if ros_det.results else 0}",
)  # 自动处理 bbox 格式、时间戳、track_id
```

**Transform 点变换（关键）**：
```python
# 错误 - Transform.apply() 用于组合变换，不是变换点
centroid_optical = world_to_optical.apply(centroid_world_vec)  # ✗

# 正确 - 使用矩阵乘法变换点
centroid_homogeneous = np.array([x, y, z, 1.0])
centroid_optical = (world_to_optical.to_matrix() @ centroid_homogeneous)[:3]  # ✓
```

**相机内参分辨率缩放（关键）**：
```python
# 错误 - 直接使用 camera_info 内参，分辨率不匹配
lcm_camera_info.K = self._camera_info.K  # ✗ 图像 320x240，内参 1280x720

# 正确 - 根据实际图像分辨率缩放
scale_x = image.width / self._camera_info.width
scale_y = image.height / self._camera_info.height
K = list(self._camera_info.K)
K[0] *= scale_x  # fx
K[2] *= scale_x  # cx
K[4] *= scale_y  # fy
K[5] *= scale_y  # cy
lcm_camera_info.K = K
lcm_camera_info.width = image.width
lcm_camera_info.height = image.height  # ✓
```

**相关文件**：
- `dimos/msgs/vision_msgs/Detection2D.py` - 消息定义
- `dimos/perception/detection/type/detection2d/bbox.py` - Detection2DBBox
- `dimos/types/timestamped.py` - `to_timestamp()` 工具

## Troubleshooting

### 居中失败 - 大角度旋转设计错误
**症状：** 导航到达后目标不在画面内，VLM 检测失败，居中被跳过

**历史错误设计：**
- 搜索阶段：90° 大角度旋转跳跃 → 每次检测间隔大，错过目标
- 居中阶段：固定角速度 → 无比例控制，容易超调或收敛慢

**正确设计（参考 person_follow）：**
- 搜索阶段：30° 小步渐进，每步检测
- 居中阶段：比例控制，偏差大→转速快，偏差小→转速慢

**关键教训：** 居中流程应参考 `person_follow._compute_warmup_centering_twist()` 的渐进式设计，而非大角度跳跃。

### 端口 5555 被占用
**错误：** `[Errno 98] error while attempting to bind on address ('0.0.0.0', 5555): address already in use`

**原因：** 之前的 dimos 进程没有完全关闭，5555 端口仍被占用。

**解决：**
```bash
pkill -f dimos
# 或查看具体进程
ps aux | grep dimos && kill <pid>
```

### 检测验证阈值
检测到的 bbox 会经过以下验证：
- **最小面积阈值**：`_navigate_to_object_min_area_ratio = 0.002` (0.2%)
- **边缘检测**：`_navigate_to_object_edge_margin_ratio = 0.03` (3%)
- 面积 < 阈值 或 在边缘的小 bbox 会被拒绝
- 日志中查看：`FOLLOW_CANDIDATE_ACCEPTED` 或 `FOLLOW_CANDIDATE_REJECTED`

### "No tracker available" Error
```bash
# Install matching opencv-contrib version
uv pip install opencv-contrib-python==$(python -c "import cv2; print(cv2.__version__)")
```

### Navigation Timeout
- Check if path planning succeeds: look for "No path found" in logs
- Try larger goal_distance (e.g., 1.0m instead of 0.5m)
- Verify object is actually reachable in the environment

### Tracking Lost Immediately
- Check `_max_stuck_frames` in `object_tracker_2d.py` (default: 100)
- CSRT may fail on very static objects - consider increasing threshold

### VLM 颜色幻觉（Color Hallucination）
**症状：** VLM 检测到带颜色前缀的目标（如"绿色瓶子"），但实际物体颜色不匹配

**案例（2026-05-15）：**
- VLM 检测 "绿色瓶子"，bbox 框选了一个深色保温杯（黑色/深灰色）
- 旁边有绿色订书机，可能导致颜色混淆
- 面积阈值和边缘检测无法过滤（bbox 大小和位置都合理）

**原因分析：**
- VLM 可能被周围物体的颜色影响
- 颜色幻觉在带颜色前缀的查询中更常见

**修复方案（待实施）：**
```python
def verify_color(image, bbox, expected_color):
    """验证 bbox 区域是否包含期望的颜色"""
    x1, y1, x2, y2 = bbox
    roi = image[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # 定义颜色范围
    color_ranges = {
        "绿色": [(35, 50, 50), (85, 255, 255)],
        "红色": [(0, 50, 50), (10, 255, 255)],
        # ...
    }
    # 检查颜色占比
    mask = cv2.inRange(hsv, *color_ranges.get(expected_color))
    return cv2.countNonZero(mask) / (roi.size / 3) > 0.3
```

### 目标检测后立即丢失（速度过快）
如果机器狗探索时速度过快，检测到目标后可能立即转走丢失：
- 使用 `DIMOS_NERF_SPEED=0.1` 或更低值启动
- 降低速度可让机器狗在检测到目标后有足够时间稳定跟踪
- 不需要修改代码，仅通过环境变量配置

### 深度估计 - 已修复（2026-05-15）
**问题：** 导航到目标只用了 1-2 秒就显示"到达"，实际距目标仍很远

**已修复：** `BBoxNavigationModule` 现在使用激光雷达点云获取真实深度

**修复方案（已实施）：**
```python
class BBoxNavigationModule(Module):
    lidar: In[PointCloud2]      # 激光雷达点云输入
    color_image: In[Image]       # 图像输入（用于时间戳匹配）
    
    def _estimate_depth_from_pointcloud(self, det: Detection2DArray):
        """使用 Detection3DPC.from_2d() 投影点云到 bbox 区域获取真实深度"""
        detection_3d = Detection3DPC.from_2d(
            det=detection_2d,
            world_pointcloud=self._latest_pointcloud,
            camera_info=lcm_camera_info,
            world_to_optical_transform=world_to_optical,
            filters=[],
        )
        # 过滤地面点 (z > 0.3m)
        # 使用质心深度
        return (x_cam, y_cam, z_cam)  # 真实深度下的相机坐标
    
    def _on_detection(self, det: Detection2DArray):
        # 优先使用点云深度
        goal_cam = self._estimate_depth_from_pointcloud(det)
        depth_source = "pointcloud"
        
        # 回退到固定深度（点云不可用时）
        if goal_cam is None:
            goal_cam = self._fixed_depth_fallback(det)
            depth_source = "fallback"
        
        # 日志显示深度来源
        logger.info(f"Goal pose: ... [depth={depth_source}]")
```

**配置参数：**
```python
use_pointcloud: bool = True   # 启用/禁用点云深度估计
goal_distance: float = 0.5    # 回退时的固定深度假设
```

**自动连接：**
- GO2Connection.lidar → BBoxNavigationModule.lidar（名称匹配）
- GO2Connection.color_image → BBoxNavigationModule.color_image（名称匹配）

**日志验证：**
- ✓ 正确：`[depth=pointcloud]` - 使用真实深度
- ⚠ 回退：`[depth=fallback]` - 使用固定深度（点云不可用）

### 检测调试图片保存
检测图片自动保存到 `~/.local/state/dimos/detection_debug/`：
- 文件名格式：`{时间}-{目标}-{状态}-{面积占比}.jpg`
- 状态：`accepted`（绿色框）或 `rejected`（红色框）
- 用于分析误检原因

```bash
# 查看检测图片
ls ~/.local/state/dimos/detection_debug/

# 用 vision_analyze 分析
# Hermes 可以直接查看这些图片
```

### 模块无输出诊断（BBoxNavigationModule 不响应）
**症状：** BBoxNavigationModule 在日志中无任何输出（无 Goal pose，无 debug 信息）

**诊断方法：**
1. 检查模块启动日志：
   ```bash
   grep "BBoxNavigationModule" ~/.local/state/dimos/logs/*/main.jsonl
   ```
   应看到：`"Deployed module."` 和 `"started successfully"`

2. 检查流连接：
   ```bash
   grep "detection2darray.*BBoxNavigationModule" ~/.local/state/dimos/logs/*/main.jsonl
   ```
   应看到：`"Transport"` 日志，说明订阅已建立

3. 添加诊断日志（修改模块代码）：
   ```python
   @rpc
   def start(self) -> None:
       logger.info("BBoxNavigationModule starting...")
       # ... subscriptions ...
       logger.info("BBoxNavigationModule started successfully")

   def _on_detection(self, det: Detection2DArray) -> None:
       logger.info(f"BBoxNavigationModule received detection: {det.detections_length} detections")
       # ... rest of method ...
   ```

**可能原因：**
- **LCM multicast 问题**：仿真环境中 LCM UDP multicast 可能失败，实机通常无此问题
- **模块启动时序**：模块可能在 publisher 发布消息前未完成订阅
- **Worker 进程异常**：模块部署成功但 worker 进程内部有异常

**验证方法：**
- 实机测试通常不会遇到 LCM multicast 问题
- 仿真环境中可检查其他模块是否能接收相同 topic（如 ObjectTracker2D 是否收到 detection2darray）

### AttributeError: 'camera_intrinsics' not found（属性重命名遗漏）
**症状：** 
```
AttributeError: 'BBoxNavigationModule' object has no attribute 'camera_intrinsics'
```

**原因：** 重命名属性时遗漏更新 `_on_detection()` 方法中的引用

**修复：** 全局搜索并替换所有引用
```python
# 错误 - 旧属性名
self.camera_intrinsics
self.camera_intrinsics is not None

# 正确 - 新属性名（存储完整 CameraInfo 消息）
self._camera_info
self._camera_info is not None
```

**验证命令：**
```bash
cd /home/emergeos/Share_pgx/ZLP/dimos
grep -n "camera_intrinsics" dimos/navigation/bbox_navigation.py
# 应返回空（无匹配）
```

### NameError: 'center_x' is not defined（变量作用域问题）
**症状：** 点云深度估算成功时抛出 NameError

**原因：** `center_x`, `center_y` 只在 fallback 分支定义，但最终日志使用

**修复：** 将变量定义移到方法开头
```python
def _on_detection(self, det: Detection2DArray) -> None:
    # ...
    
    # Get detection center for logging (used in both paths)
    center_x = det.detections[0].bbox.center.position.x
    center_y = det.detections[0].bbox.center.position.y
    
    # Try pointcloud-based depth estimation first
    goal_cam = self._estimate_depth_from_pointcloud(det)
    
    # Fallback (center_x, center_y already defined above)
    if goal_cam is None:
        # ...
```

### source_frame 坐标系不匹配
**症状：** TF 变换失败，目标被当作原点坐标

**原因：** 使用 `det.header.frame_id`（通常是 `camera_optical`），但 `goal_cam` 是 `camera_link` 坐标

**修复：** 固定使用正确的源坐标系
```python
# 错误 - 使用检测消息的 frame_id
source_frame = det.header.frame_id  # 可能是 "camera_optical"

# 正确 - goal_cam 总是在 camera_link 坐标系
source_frame = "camera_link"
```

### Tracker 过早失败
**症状：** Tracker 只追踪几秒就失败，导航继续使用初始目标位置

**案例（2026-05-15）：**
- Tracker 初始化成功，但 0.7 秒后失败
- 导航继续使用 bbox 中心计算的初始目标 (0.50, 0.07)
- 可能导致导航到错误位置

**排查：**
```bash
# 查看日志中的 Tracker 事件
grep "Tracker update" ~/.local/state/dimos/logs/*/main.jsonl
```

**可能原因：**
- CSRT tracker 对静止物体敏感度低
- 物体移出视野太快
- 光照变化

## Usage Patterns

### Pattern 1: Target Already in View
当目标已在相机视野内时，直接调用：
```bash
dimos mcp call navigate_to_object --json-args '{"query": "wall", "goal_distance": 1.0}'
```

### Pattern 2: Search and Navigate (目标位置未知)
当目标位置未知时，需要使用**循环策略**：探索 → 检测 → 未找到 → 继续探索 → 循环

```python
# Agent 控制的循环策略
import subprocess, time

dimos = "/path/to/dimos/.venv/bin/dimos"
max_attempts = 10

for attempt in range(max_attempts):
    # 启动探索
    subprocess.run([dimos, "mcp", "call", "begin_exploration"])
    
    # 探索一段时间让机器人移动到新区域
    time.sleep(10)
    
    # 尝试检测目标
    result = subprocess.run([dimos, "mcp", "call", "navigate_to_object", 
        "--json-args", '{"query": "chair", "goal_distance": 1.5}'],
        capture_output=True, text=True)
    
    if "Successfully arrived" in result.stdout:
        print("成功导航到目标")
        break
    elif "not found" in result.stdout:
        print("目标未在视野内，继续探索...")
        subprocess.run([dimos, "mcp", "call", "end_exploration"])
        time.sleep(1)
```

**验证结果（2026-05-14）：**
- 第3次尝试成功找到椅子并导航到面前
- 墙壁导航：第1次尝试成功

## Examples

### Navigate to Wall (已在视野)
```bash
dimos mcp call navigate_to_object --json-args '{"query": "wall", "goal_distance": 1.0}'
# Output: Successfully arrived near 'wall'
```

### Navigate to Chair (需要搜索)
使用循环策略，探索10秒后检测，重复直到找到。

## Related Skills
- `dimos-explore-find-navigate` - For dynamic targets (people) with auto-follow
- `look_out_for` - Continuous detection with trigger actions
- `follow_person` - Person tracking and following