# Isaac Sim Communication Configuration

本文档说明如何配置 Contact-GraspNet 与 Isaac Sim 的通信。

## 通信架构

```
┌─────────────────────────────────────────────────────────────────┐
│ 远端 (192.168.100.12)                                           │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐       │
│  │ Isaac    │───▶│ ROS2     │───▶│ remote_forwarder.py  │       │
│  │ Sim      │    │ Topics   │    │ (ZMQ PUB:5555)       │       │
│  │          │◀───│          │◀───│ (ZMQ SUB:5556)       │       │
│  └──────────┘    └──────────┘    └──────────────────────┘       │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    │ ZMQ TCP
                                    │
┌─────────────────────────────────────────────────────────────────┐
│ 本地                                                            │
│  ┌──────────────────────┐    ┌──────────────────────────────┐   │
│  │ IsaacSimClient       │◀──▶│ Contact-GraspNet             │   │
│  │ (ZMQ SUB:5555)       │    │ GraspEstimator               │   │
│  │ (ZMQ PUB:5556)       │    │                              │   │
│  └──────────────────────┘    └──────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## 远端配置 (192.168.100.12)

### 1. 安装依赖

```bash
# 确保 ROS2 已安装
# Ubuntu 22.04: ROS2 Humble
# Ubuntu 24.04: ROS2 Jazzy

# 安装 Python 依赖
pip install pyzmq msgpack msgpack-numpy scipy
```

### 2. Isaac Sim ROS2 桥接

在 Isaac Sim 中启用 ROS2 桥接，确保发布以下话题：

| ROS2 Topic | 类型 | 描述 |
|------------|------|------|
| `/piper/end_cam/depth_image` | `sensor_msgs/Image` | 深度图 (32FC1 或 16UC1) |
| `/piper/end_cam/color_image` | `sensor_msgs/Image` | RGB 图像 |
| `/piper/end_cam/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 (可选) |
| `/piper/end_cam/segmentation` | `sensor_msgs/Image` | 实例分割图 (可选) |

### 3. 运行转发器

```bash
# SSH 连接
ssh user@192.168.100.12
# 密码: Pass5157

# 运行转发器
cd /path/to/contact_graspnet_pytorch-main
python3 tools/contact_graspnet_remote_forwarder.py \
    --sensor-port 5555 \
    --grasp-port 5556
```

如果你只是做原始 RGB/Depth 抓取，远端无需提供分割图。本地推理脚本现在支持直接对 RGB 运行 SAM 生成实例分割。

只有在你明确想兼容外部分割源时，才需要转发实例分割图：

```bash
python3 tools/contact_graspnet_remote_forwarder.py \
    --sensor-port 5555 \
    --grasp-port 5556 \
    --enable-segmap \
    --segmap-topic /piper/end_cam/segmentation
```

如果话题名称不同，可以直接通过参数覆盖：

```bash
python3 tools/contact_graspnet_remote_forwarder.py \
    --depth-topic /isaac/depth \
    --rgb-topic /isaac/rgb \
    --camera-info-topic /isaac/camera_info \
    --enable-segmap \
    --segmap-topic /isaac/segmentation
```

## 本地配置

### 1. 运行抓取推理

```bash
# 激活环境
conda activate contact_graspnet

# 如果使用本地 SAM 分割，先安装依赖
pip install segment-anything

# 如果需要按名称选择目标实例，再安装文本选择依赖
pip install transformers

# 单帧推理
python tools/inference_isaac_sim.py --remote-ip 192.168.100.12

# 连续推理模式
python tools/inference_isaac_sim.py --remote-ip 192.168.100.12 --continuous

# 连续推理 + 下发可执行抓取命令
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --execute-best-grasp \
    --min-grasp-score 0.25 \
    --pregrasp-offset 0.12 \
    --retreat-offset 0.10

# 使用本地 SAM 实例分割约束抓取，只处理前景实例
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --use-segmap \
    --segmentation-source sam \
    --sam-checkpoint /path/to/sam_vit_b_01ec64.pth \
    --local-regions \
    --filter-grasps

# 只抓指定实例 ID，并在执行前做工作空间过滤
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --use-segmap \
    --segmentation-source sam \
    --sam-checkpoint /path/to/sam_vit_b_01ec64.pth \
    --segmap-id 3 \
    --local-regions \
    --filter-grasps \
    --execute-best-grasp \
    --workspace-bounds "[[-0.20,0.20],[-0.10,0.45],[0.20,0.70]]" \
    --execution-top-k 10

# 按名称抓取某类物体：SAM 分实例 -> CLIP 在实例里选择最像 "cup" 的目标 -> 只对该实例抓取
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --segmentation-source sam \
    --sam-checkpoint /path/to/sam_vit_b_01ec64.pth \
    --target-query cup \
    --target-selector clip \
    --target-selector-model openai/clip-vit-base-patch32 \
    --local-regions \
    --filter-grasps \
    --execute-best-grasp \
    --workspace-bounds "[[-0.20,0.20],[-0.10,0.45],[0.20,0.70]]" \
    --execution-top-k 10

# 保存 RGB / 目标 mask / 有效深度 overlay，排查 RGB-Depth 对齐与目标点云过少问题
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --segmentation-source sam \
    --sam-checkpoint /path/to/sam_vit_b_01ec64.pth \
    --target-query bottle \
    --debug-save-dir debug/segmentation \
    --debug-save-every 10

# 如果确认只是 segmap/RGB 与 depth 分辨率不同，可临时打开最近邻重采样验证
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --segmentation-source sam \
    --sam-checkpoint /path/to/sam_vit_b_01ec64.pth \
    --target-query bottle \
    --resize-segmap-to-depth \
    --debug-save-dir debug/segmentation

# 仅在兼容旧链路时，才使用远端转发的 segmap
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --continuous \
    --use-segmap \
    --segmentation-source remote \
    --local-regions \
    --filter-grasps
```

### 2. 自定义配置

```bash
# 自定义端口
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --sensor-port 5555 \
    --grasp-port 5556

# 自定义 Z 范围
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --z-range "[0.1, 1.5]"

# 禁用可视化 (提高速度)
python tools/inference_isaac_sim.py \
    --remote-ip 192.168.100.12 \
    --no-visualize \
    --continuous
```

## 数据格式

### 发送到本地的数据 (SensorData)

```python
{
    'depth': np.ndarray,     # HxW 深度图 (单位: 米)
    'rgb': np.ndarray,       # HxWx3 RGB 图像
    'K': np.ndarray,         # 3x3 相机内参矩阵
    'segmap': np.ndarray,    # HxW 实例分割图 (可选)
    'timestamp': float,      # 时间戳 (秒)
    'frame_id': int          # 帧序号
}
```

### segmap 格式要求

`segmap` 必须是和深度图严格对齐的单通道整数实例图：

- 形状：`HxW`
- 类型：`np.int32`、`np.uint16` 等整数类型
- 语义：`0` 表示背景，`>0` 表示不同实例 ID
- 不是 RGB mask，也不是 one-hot

最小示例：

```python
segmap = np.zeros((H, W), dtype=np.int32)
segmap[obj_a_mask] = 1
segmap[obj_b_mask] = 2
```

只有本地推理使用 `--use-segmap` 时，程序才会走基于实例分割的 `predict_scene_grasps(...)` 路径；否则仍然会退回整场景抓取。

### 推荐分割来源

默认推荐：

1. 原始 `rgb/depth/K`
2. 本地 SAM 自动分割得到实例 `segmap`
3. 如果有目标名称，用文本选择器在这些实例里找出目标实例
4. `predict_scene_grasps(...)`
5. 工作空间和 top-k 过滤后再执行

不推荐默认依赖 Isaac Sim 的语义/实例分割结果作为抓取输入，除非你只是做兼容性测试或已有成熟的外部分割节点。

### 按名称抓取的工作方式

当前实现的“按名称抓取”流程是：

1. 用 SAM 生成实例分割
2. 把每个实例裁成 masked RGB crop
3. 用文本-图像模型对这些实例打分
4. 选出最匹配 `--target-query` 的实例
5. 只对该实例运行 Contact-GraspNet 并执行抓取

当前内置的文本选择后端是 `clip`，适合当前这种“同一场景里若干未知实例，按名称挑一个”的需求。它不是检测框模型，而是“在 SAM 已分好的实例里做文本匹配”。

### 分割/深度对齐调试

当目标实例在点云中只有极少点时，优先打开：

- `--debug-save-dir`
- `--debug-save-every`
- `--resize-segmap-to-depth`（仅用于验证是否是纯分辨率不一致）

脚本会保存：

- 原始 RGB 图
- 目标实例 mask
- 有效深度 mask
- 叠加图
- 如果启用重采样，还会保存对齐后的 RGB overlay 版本

其中：

- 绿色：目标实例 mask
- 蓝色：有效深度区域
- 黄色：目标 mask 与有效深度的重叠区域

### 抓取选择逻辑

当前推理脚本支持两层筛选：

1. 候选生成：
   使用整场景点云，或在 `--use-segmap` 下使用实例分割裁出的 `pc_segments`
2. 执行选择：
   对预测结果按分数排序，并可附加：
   - `--segmap-id`：只从指定实例选抓取
   - `--workspace-bounds`：只保留给定执行坐标系下的空间范围
   - `--execution-top-k`：只在前 `k` 个高分候选中选执行抓取

### 发送回远端的数据 (GraspResult)

```python
{
    'grasp_poses': dict[int, np.ndarray],  # Nx4x4 抓取位姿矩阵
    'scores': dict[int, np.ndarray],       # N 置信度分数
    'contact_points': dict[int, np.ndarray], # Nx3 接触点
    'gripper_openings': dict[int, np.ndarray], # N 夹爪开口宽度
    'timestamp': float,
    'frame_id': int,
    'status': str,          # 'success', 'no_grasp', 'error'
    'message': str,
    'execution': {          # 可选，最佳抓取的执行命令
        'pose': np.ndarray,           # 4x4 抓取位姿
        'pregrasp_pose': np.ndarray,  # 4x4 预抓取位姿
        'retreat_pose': np.ndarray,   # 4x4 撤离位姿
        'contact_point': np.ndarray,  # 3D 接触点
        'approach_vector': np.ndarray,# 3D 单位接近向量
        'gripper_opening': float,     # 夹爪开口
        'score': float,               # 该抓取分数
        'segment_id': int,
        'frame': str,                 # 坐标系名，例如 camera_optical_frame
        'pregrasp_offset': float,
        'retreat_offset': float
    }
}
```

## 远端执行话题

当本地使用 `--execute-best-grasp` 时，远端转发器会额外发布以下 ROS2 话题，供 Isaac Sim 脚本或机械臂控制节点订阅：

| ROS2 Topic | 类型 | 描述 |
|------------|------|------|
| `/piper/pregrasp_pose` | `geometry_msgs/PoseStamped` | 预抓取位姿 |
| `/piper/target_grasp` | `geometry_msgs/PoseStamped` | 抓取位姿 |
| `/piper/postgrasp_pose` | `geometry_msgs/PoseStamped` | 抓后撤离位姿 |
| `/piper/gripper_opening` | `std_msgs/Float32` | 建议夹爪开口宽度 |
| `/piper/execute_grasp` | `std_msgs/Bool` | 执行触发信号 |
| `/piper/grasp_execution` | `std_msgs/String` | 完整 JSON 执行命令 |

## 执行节点 (grasp_execution_node.py)

`grasp_execution_node.py` 是一个 ROS2 节点，运行在远端机器上，用于执行抓取命令。

### 功能

1. 订阅抓取执行话题 (`/piper/grasp_execution` 或单独的位姿话题)
2. 获取当前末端执行器位姿 (`/piper/end_pose`)
3. 将相机光学坐标系下的抓取位姿变换到机器人基座坐标系
4. 调用 MoveIt IK 服务 (`/compute_ik`) 计算关节角度
5. 发布关节控制命令 (`/piper/joint_ctrl`) 执行运动
6. 执行完整的预抓取 -> 抓取 -> 撤离动作序列

### 坐标变换配置

```
坐标变换链: camera_frame -> gripper_base_frame -> world_frame -> base_link

相机相对于 gripper_base:
- translation: [0.0, 0.0, 0.12] (12cm 向前)
- rotation: euler [0, -90, 0] (pitch -90°, 指向前方)

机器人基座在世界坐标系:
- position: [0.4, 0.35, 0.85]
- rotation: 无旋转
```

### 运行执行节点

```bash
# SSH 连接远端
ssh jjl@192.168.100.12

# 激活 isaaclab 环境
conda activate isaaclab

# 确保 MoveIt IK 服务正在运行
ros2 service list | grep compute_ik

# 运行执行节点
cd /path/to/contact_graspnet_pytorch-main/tools
python3 grasp_execution_node.py

# 如果使用简化执行器，并希望在 IK 前过滤掉超出机械臂基座工作空间的目标
python3 grasp_executor_simple.py \
    --workspace-bounds "[[-0.10,0.35],[-0.10,0.35],[0.05,0.45]]"
```

### 执行流程

当收到执行命令时，执行节点按以下顺序执行:

1. **打开夹爪** - 发布 `/piper/gripper_command` (0.08)
2. **移动到预抓取位姿** - 调用 IK 并发布关节轨迹
3. **移动到抓取位姿** - 调用 IK 并发布关节轨迹
4. **闭合夹爪** - 发布 `/piper/gripper_command` (指定宽度)
5. **移动到撤离位姿** - 调用 IK 并发布关节轨迹

### 工作空间过滤建议

`z_range` 只是在相机深度里裁点云，不等于机械臂可达空间。对真正的可执行性，更有效的是在执行器的 `planning frame` 里加工作空间盒约束。

对当前 `grasp_executor_simple.py`，`--workspace-bounds` 是在 MoveIt 规划帧中生效，也就是机械臂基座附近的可达盒，而不是相机光学系。

### 话题接口

| ROS2 Topic | 方向 | 类型 | 描述 |
|------------|------|------|------|
| `/piper/grasp_execution` | IN | `std_msgs/String` | JSON 执行命令 (主要接口) |
| `/piper/pregrasp_pose` | IN | `geometry_msgs/PoseStamped` | 预抓取位姿 (备用) |
| `/piper/target_grasp` | IN | `geometry_msgs/PoseStamped` | 抓取位姿 (备用) |
| `/piper/postgrasp_pose` | IN | `geometry_msgs/PoseStamped` | 撤离位姿 (备用) |
| `/piper/gripper_opening` | IN | `std_msgs/Float32` | 夹爪开口宽度 |
| `/piper/execute_grasp` | IN | `std_msgs/Bool` | 执行触发信号 |
| `/piper/end_pose` | IN | `geometry_msgs/PoseStamped` | 当前末端位姿 |
| `/piper/joint_ctrl` | OUT | `trajectory_msgs/JointTrajectory` | 关节控制命令 |
| `/piper/gripper_command` | OUT | `std_msgs/Float32` | 夹爪控制命令 |

### MoveIt 配置

执行节点需要 MoveIt IK 服务。请确保:

1. MoveIt 配置文件中定义了 `piper_arm` 规划组
2. IK 服务 `/compute_ik` 正在运行
3. 关节名称列表正确: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`

如果关节名称不同，需要修改 `grasp_execution_node.py` 中的 `joint_names` 列表。

## 网络配置

### 防火墙设置

确保端口 5555 和 5556 在远端机器上开放：

```bash
# 在远端执行 (192.168.100.12)
sudo ufw allow 5555/tcp
sudo ufw allow 5556/tcp
```

### 测试连接

```bash
# 在本地测试 TCP 连接
nc -zv 192.168.100.12 5555
nc -zv 192.168.100.12 5556
```

## 常见问题

### 1. 无法连接

- 检查远端 IP 是否正确
- 检查防火墙是否开放端口
- 检查转发器是否在远端运行

### 2. 没有收到数据

- 检查 Isaac Sim 是否正在发布 ROS2 话题
- 检查 ROS2 话题名称是否匹配
- 使用 `ros2 topic list` 确认话题

### 3. ROS2 话题不匹配

编辑 `remote_forwarder.py` 修改话题名称：

```python
'/isaac/depth' -> '/camera/depth/image_raw'
'/isaac/rgb' -> '/camera/rgb/image_raw'
'/isaac/camera_info' -> '/camera/camera_info'
```

## 直接使用 Python API

```python
from contact_graspnet_pytorch.comm import IsaacSimClient, SensorData, GraspResult
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator

# 加载模型
grasp_estimator = GraspEstimator(config)

# 连接远端
client = IsaacSimClient(remote_ip="192.168.100.12")
client.connect()

# 接收数据
sensor_data = client.wait_for_data(timeout_s=5.0)

# 提取点云并预测
pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
    sensor_data.depth, sensor_data.K, segmap=sensor_data.segmap
)

pred_grasps, scores, contact_pts, openings = grasp_estimator.predict_scene_grasps(pc_full, pc_segments)

# 发送结果
client.send_grasp_from_prediction(pred_grasps, scores, contact_pts, openings, frame_id=sensor_data.frame_id)

# 断开连接
client.disconnect()
```
