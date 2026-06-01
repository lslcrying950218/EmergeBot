---
name: explore-find-navigate
title: DimOS 探索-发现-跟踪 Tool 链
version: 4.0.0
description: Agent-driven 方式控制 DimOS 机械狗自主探索、发现目标（人/物体）并自动跟踪/导航。支持静态物体导航（navigate_to_object）和动态目标跟踪（follow_person）。新增 auto_explore 参数，一个命令即可启动探索+检测。
triggers:
  - "机械狗探索"
  - "自主探索"
  - "寻找并跟踪"
  - "找.*跟踪"
  - "探索.*找.*人"
  - "机器人.*巡逻"
  - "dog explore"
  - "robot explore and track"
  - "find and follow person"
  - "explore.*find.*follow"
  - "dimos explore"
  - "让机械狗"
  - "跟踪目标"
  - "丢失后继续"
  - "导航到"
  - "移动到.*面前"
  - "navigate to"
  - "go to"
  - "搜索.*移动"
  # 抓取/拿起相关触发词
  - "抓取"
  - "拿起"
  - "抓.*瓶子"
  - "抓.*杯子"
  - "找.*抓取"
  - "寻找.*抓"
  - "搜索.*抓"
  - "并抓取"
  - "并拿起"
  - "抓起来"
  - "pick up"
  - "grab"
  - "find and pick"
  - "search and grab"
---

# DimOS 探索-发现-跟踪 Tool 链

## 使用场景

需要让 Unitree Go2（模拟或真机）自主在环境中移动，寻找特定目标（如人），并在发现后自动开始持续跟踪，丢失后自动重新探索查找。

**核心特点：**
- Agent-driven：agent 逐轮读取日志、判断状态、做出决策，对动态变化更灵敏
- 备选脚本：无人值守时可用 `scripts/log_monitor_loop.py`
- **自然语言输入**：用户可直接说"寻找水瓶并停在它面前"，Hermes 自动解析生成正确的 MCP 命令

## 自然语言指令（Hermes 解析）

用户可以直接输入自然语言指令，Hermes 自动解析并生成正确的 MCP 命令：

| 用户指令 | Hermes 生成的命令 |
|---------|------------------|
| "寻找水瓶并停在它面前" | `look_out_for(description_of_things=["水瓶","water bottle"], auto_explore=true, then=navigate_to_object)` |
| "找可乐然后导航过去" | `look_out_for(description_of_things=["可乐","cola"], auto_explore=true, then=navigate_to_object)` |
| "找穿灰色衣服的人并跟踪" | `look_out_for(description_of_things=["穿灰色衣服的人"], auto_explore=true, then=follow_person)` |
| "搜索矿泉水瓶，找到后停在那里" | `look_out_for(description_of_things=["矿泉水瓶","water bottle"], auto_explore=true, then=navigate_to_object)` |
| "寻找绿色瓶子并抓取" | `look_out_for(description_of_things=["绿色瓶子","green bottle"], auto_explore=true, then={navigate_to_object, then=pick})` |
| "找水瓶并抓起来" | `look_out_for(description_of_things=["水瓶","water bottle"], auto_explore=true, then={navigate_to_object, then=pick})` |
| "搜索瓶子拿起它" | `look_out_for(description_of_things=["瓶子","bottle"], auto_explore=true, then={navigate_to_object, then=pick})` |

**解析规则：**
- "寻找/找/搜索" → 启动检测
- "停在/移动到/导航到" → 静态物体，用 `navigate_to_object`
- "跟踪/跟随" → 动态目标，用 `follow_person`
- **"抓取/拿起/抓起来/捡" → 必须在 navigate_to_object 后追加 pick，用嵌套 then 格式**
- 自动设置 `auto_explore=true` 实现边移动边检测

**⚠️ 关键：当用户说"抓取/拿起"时，必须生成三层嵌套 then 链！**

用户说"寻找X并抓取"或"找X并拿起"时，必须生成完整的三层链：
```
look_out_for → then: navigate_to_object → then: pick
```

如果只生成 `look_out_for → then: navigate_to_object`（缺少 pick），机器人只会导航到目标面前而不会抓取，**这是错误的**。

**正确格式示例：**
```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["绿色瓶子", "green bottle"],
  "auto_explore": true,
  "then": {
    "name": "navigate_to_object",
    "arguments": {"query": "绿色瓶子", "goal_distance": 0.5},
    "then": {
      "name": "pick",
      "arguments": {}
    }
  }
}'
```

## 操作模式

根据用户意图选择合适的模式：

### 模式 1：纯检测（不跟踪）

**触发词：** "检测"、"查看"、"是否有"、"看看"、"有没有"

当用户只想知道某物是否存在，不需要后续动作时使用：

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"]}'
```

返回：检测结果（是否找到、位置等），不会自动跟踪。

### 模式 2：检测并自动跟踪

**触发词：** "跟踪"、"跟随"、"找到并跟踪"、"跟着"

当用户希望发现目标后自动开始跟踪时使用：

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"], "then": {"name": "follow_person", "arguments": {"query": "person"}}}'
```

返回：检测启动成功，`then` 参数确保发现目标后自动触发 `follow_person`

**⚠️ 常见错误 - 参数格式不正确：**

```bash
# ❌ 错误：target 不是有效参数（应该是 description_of_things）
dimos mcp call look_out_for --arg "target:穿凉鞋的人" --arg "then:follow_person"

# ❌ 错误：then 需要是完整的 tool call 对象
dimos mcp call look_out_for --json-args '{"target": "穿凉鞋的人", "then": "follow_person"}'
# 错误信息：PerceiveLoopSkill.look_out_for() got an unexpected keyword argument 'target'

# ✅ 正确：使用 description_of_things 数组，then 包含完整工具调用
dimos mcp call look_out_for --json-args '{"description_of_things": ["穿凉鞋的人"], "then": {"name": "follow_person", "arguments": {"query": "穿凉鞋的人"}}}'
# 返回：Started looking for [...]. Continuation logic armed.
```

**参数说明：**
- `description_of_things`: 目标列表，用数组格式 `["目标 1", "目标 2"]`
- `query` (在 then 内部): `follow_person` 使用的查询字符串
- `then`: 必须包含 `name`（工具名）和 `arguments`（工具参数）

### 模式 3：导航到静态物体

**触发词：** "导航到"、"移动到"、"去"、"走到"、"搜索并移动"

当用户希望机器人导航到静态物体（如墙、书架、桌子、箱子等）面前时使用：

```bash
dimos mcp call navigate_to_object --json-args '{"query": "wall", "goal_distance": 1.5}'
```

返回：导航启动消息（导航在后台运行）

**参数说明：**
- `query`: 目标物体的自然语言描述（如 "wall", "bookshelf", "water bottle"）
- `goal_distance`: 到达目标后与目标的距离（米），推荐值 1.5m（太近可能导致障碍物检测问题）

**⚠️ 导航架构（简化版，2026-05-15）：**
```
look_out_for 检测到目标 → 面积/边缘验证 → 直接用 initial_bbox 启动 CSRT 追踪 → BBox→3D 投影 → A* 路径规划 → cmd_vel
```
检测到目标后直接用 initial_bbox 启动追踪和导航，不再经过等待稳定、重新检测、居中等中间步骤。

**关键修复：**
- 移除了 0.5s 等待（目标可能在这段时间移出视野）
- 移除了重新检测（VLM 调用增加延迟，容易丢失目标）
- 移除了居中逻辑（多步 VLM 检测 + cmd_vel 转向，累积失败率高）
- 保留 `cancel_goal() + Twist.zero()` 停止探索惯性

**注意：**
- 静态物体导航使用 `navigate_to_object`（依赖 ObjectTracker2D + BBoxNavigationModule）
- 动态目标跟踪使用 `look_out_for` + `follow_person`（依赖 PersonFollowSkillContainer）
- 两者可以在同一个 `unitree-go2-agentic` blueprint 中同时使用
- **`navigate_to_object` 现在异步返回**（2026-05-14 修复），不阻塞检测循环

### 模式 4：探索 + 检测 + 自动导航到静态物体（推荐使用 auto_explore）

**触发词：** "找到椅子并导航过去"、"搜索墙并移动到前面"、"搜索水瓶并停在它面前"

当用户希望机器人边探索边检测静态物体，发现后自动导航过去时使用。

**推荐方式（auto_explore=true，一个命令完成）：**

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["椅子", "chair"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "椅子", "goal_distance": 1.5}}}'
```

**auto_explore 参数说明：**
- `auto_explore: true` — 自动启动 `begin_exploration`，机器人开始移动寻找目标
- 检测到目标后，探索自动停止，导航立即启动
- 调用 `stop_looking_out` 时，由 auto_explore 启动的探索也会自动停止

### 模式 5：探索 + 检测 + 导航 + 抓取（完整闭环）

**触发词：** "找到瓶子并抓取"、"搜索水瓶拿起它"、"找绿色瓶子并抓起来"

当用户希望机器人完成完整的"探索→检测→导航→抓取"闭环时使用。这是当前最完整的自动化任务链。

**链式 continuation 格式（then 里嵌套 then）：**

```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["绿色瓶子", "green bottle", "water bottle"],
  "auto_explore": true,
  "then": {
    "name": "navigate_to_object",
    "arguments": {"query": "绿色瓶子", "goal_distance": 0.5},
    "then": {
      "name": "pick",
      "arguments": {}
    }
  }
}'
```

**链式 continuation 工作原理：**

1. `look_out_for` 启动检测循环 + 自动探索
2. VLM 检测到目标 → **立即停止探索**（在 `_handle_match` 中）→ 触发 `navigate_to_object`
3. 导航成功 → **停止检测循环** → 自动触发 `pick`
4. 机械臂抓取 → 抓取成功 → 任务完成

**参数传递：**
- `initial_bbox`: 检测阶段的目标 bbox，传递给 `navigate_to_object` 和 `pick`
- `initial_image`: 检测阶段的原始图像（base64）
- `label`: 检测到的目标名称（如"绿色瓶子"），自动作为 `pick` 的 `object_name`

**⚠️ 注意事项：**
- `goal_distance` 建议设置较小值（如 0.5m），确保机器人足够接近目标便于机械臂抓取
- 探索在检测确认后立即停止（不是在导航完成后）
- 检测循环在导航成功后停止（一次性任务）

**🔧 技术实现细节（供开发者参考）：**

| 阶段 | 停止时机 | 代码位置 |
|-----|---------|---------|
| 探索 | 检测确认时 | `_handle_match` 中调用 `exploration.end_exploration()` |
| 检测循环 | navigate_to_object 成功后 | `_execute_continuation` 中调用 `stop_looking_out` |

**常见误区：**
- ❌ 在 navigate_to_object 成功后调用 `end_exploration`（探索已在检测时停止）
- ❌ 在 pick 成功后调用 `end_exploration`（探索早已停止）
- ✅ 探索停止时机：检测确认 → 检测循环停止时机：导航成功

**手动方式（需同时调用两个命令）：**

```bash
# 旧方式：需要两个命令
dimos mcp call begin_exploration
dimos mcp call look_out_for --json-args '{"description_of_things": ["椅子", "chair"], "then": {"name": "navigate_to_object", "arguments": {"query": "椅子"}}}'
```

**⚠️ 已修复的关键问题（2026-05-14）：**

`navigate_to_object` 曾是同步阻塞调用，会等待导航完成（最多30秒），导致检测循环无法继续。现已修改为异步返回，启动导航后立即返回，检测循环可以继续寻找其他目标。

**技术细节：**
- `_FOLLOW_CONTINUATION_TOOLS` 列表已包含 `"navigate_to_object"`
- `_augment_follow_continuation_args` 会自动注入 `initial_bbox` 和 `initial_image`
- `navigate_to_object` 函数签名已更新以接受这些可选参数

## 前置条件

1. DimOS 正在运行且包含 `McpServer` 的 blueprint：
   ```bash
   # 推荐启动命令（仿真模式，慢速便于检测）
   DIMOS_NERF_SPEED=0.1 dimos --simulation --viewer none run unitree-go2-agentic --daemon
   
   # 或真机
   dimos run unitree-go2-agentic --robot-ip 192.168.123.161
   ```
2. MCP 服务可访问（默认 `http://localhost:9990/mcp`）

## 已验证的功能（2026-05-14 测试通过）

- ✓ `auto_explore=true` 自动启动探索，检测到目标后探索自动停止
- ✓ 探索在 `navigate_to_object` 调用时正确停止（EXPLORATION_STOPPED 日志出现）
- ✓ VLM 检测静态物体（如鞋子、水瓶、纸箱子）成功
- ✓ BBoxNavigationModule 自动生成导航目标，position_threshold 生效（0.2m）
- ✓ 小物体检测通过（area_ratio 0.0076，min_threshold 0.0005）
- ✓ 导航完成后机器人保持静止（无继续移动问题）
- ✓ 导航耗时约 4-14 秒（速度设置 0.1）
- ✓ **导航完成后检测循环自动停止**（2026-05-14 新修复）
  - `navigate_to_object` 是一次性任务，触发后立即停止检测循环
  - `follow_person` 是持续跟踪，检测循环继续运行以更新 bbox
  - 验证日志：`STATIC_NAVIGATION_COMPLETE: Stopping detection loop`
- ✓ **探索竞争条件已修复**（2026-05-14 新修复）
  - 探索循环在 `end_exploration()` 后不再发布新目标
  - 发布目标前检查 `stop_event`，已停止则立即退出
  - 验证日志：`Exploration stopped before publishing goal`

### 环境依赖

- **turbojpeg 库**（必须）：`follow_person` 技能依赖 `turbojpeg` 进行图像解码
  - 如果缺失，`follow_person` 会报错：`Unable to locate turbojpeg library automatically`
  - **即使目标检测成功也会无法启动跟踪**
  - 解决方案：
    ```bash
    sudo apt-get install libturbojpeg0-dev
    pip install turbojpeg
    ```
  - 或安装替代：`pip install pylibjpeg pylibjpeg-libjpeg`

- **dimos 命令路径**：`dimos` 可能不在 PATH 中，需要使用完整路径
  - 检查：`which dimos`
  - 如果未找到，使用：`/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos`
  - 建议将 dims 添加到 PATH：`export PATH=$PATH:/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin`

## 常见问题排查

### follow_person 失败但检测成功

**症状**：
- 日志中出现 `POLLING_MATCH: Detected candidate` 或 `POLLING_MATCH: Verified [...]`
- 但随后出现 `ERROR: Exception in RPC handler for PersonFollowSkillContainer/follow_person`
- 错误信息：`Unable to locate turbojpeg library automatically`

**原因**：缺少 `turbojpeg` Python 库或系统库

**解决步骤**：
1. 检查库是否安装：
   ```python
   import turbojpeg
   ```
2. 安装系统依赖：
   ```bash
   sudo apt-get install libturbojpeg0-dev libjpeg-dev
   ```
3. 安装 Python 包：
   ```bash
   pip install turbojpeg
   # 或
   pip install pylibjpeg pylibjpeg-libjpeg
   ```
4. 重启 DimOS：
   ```bash
   dimos restart
   ```

### MCP 命令超时或找不到命令

**症状：**
- `dimos mcp call ...` 返回 `[Command timed out after 30s]`
- 或 `/usr/bin/bash: 行 3: dimos: 未找到命令`
- 或启动时 `ERROR: [Errno 98] error while attempting to bind on address ('0.0.0.0', 5555): address already in use`

**解决：**
1. **端口被占用**（5555 被之前进程占用）：
   ```bash
   pkill -f dimos
   # 或查看具体进程
   ps aux | grep dimos && kill <pid>
   ```
2. 使用完整路径调用：
   ```bash
   /path/to/dimos/.venv/bin/dimos mcp call ...
   ```
3. 或先检查进程状态：
   ```bash
   /path/to/dimos/.venv/bin/dimos status
   ```
4. 如果进程未运行，先启动：
   ```bash
   /path/to/dimos/.venv/bin/dimos --simulation run unitree-go2-agentic --daemon
   ```

### exploration 停止但检测仍在运行

**现象**：
- 日志中出现 `EXPLORATION_STOPPED: Autonomous frontier exploration has ENDED`
- 但 `look_out_for` 仍在继续检测
- 机器人停在当前位置

**解决**：只需重启 exploration，无需重启 look_out_for
```bash
dimos mcp call begin_exploration
```

### 目标被多次检测但跟踪未启动

**可能原因：**
1. turbojpeg 库缺失（最常见）
2. EdgeTAM 初始化失败
3. SAM2 权重未下载

**诊断：**
- 查看日志中是否有 `follow_person called, awaiting execution` 但随后出现 `ERROR`
- 如果有，通常是 turbojpeg 问题
- 如果是首次运行，可能需要联网下载 SAM2 权重

**注意：** `look_out_for` 的检测循环独立运行，即使 `follow_person` 失败，检测会继续进行下一次尝试。

### VLM 检测结果为空（raw_response: "[]"）

**症状：**
- 日志中 `VLM_RAW_RESPONSE: Got response from VLM` 显示 `raw_response: "[]"`
- `VLM_NO_DETECTION: No detections for query` 连续出现
- `POLLING_QUERY_RESULT: Detection query finished` 显示 `detection_count: 0`

**原因：** 目标不在机器人相机视野内，机器人静止不动无法发现目标

**解决方案（推荐）：** 使用 `auto_explore: true` 参数，一个命令同时启动探索和检测：

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["water bottle"], "auto_explore": true, "then": {...}}'
```

**手动方式：**
1. 确认目标确实存在于环境中
2. 同时启动探索和检测：
   ```bash
   dimos mcp call begin_exploration
   dimos mcp call look_out_for --json-args '{"description_of_things": ["person"]}'
   ```

**注意：** `look_out_for` 默认只做检测，不会主动让机器人移动。必须配合 `begin_exploration` 或使用 `auto_explore: true` 才能搜索视野外的目标。

### 导航固定移动到错误位置（TF 坐标变换失败）

**症状：**
- 检测到目标后，机器人总是导航到相同的固定位置（约 `(0.30, -0.xx, 0.xx)`）
- 无论机器人在哪个位置启动，导航目标都接近原点
- 日志显示 `Could not get transform from camera_link to odom`

**根因分析（双重 bug）：**

1. **TF 树构建问题**：实机 odom 消息没有设置 `frame_id`，导致 TF 树中的 parent frame 为空
2. **Fallback 逻辑错误**：当 TF 变换失败时，代码错误地将相机坐标系相对坐标直接当作全局坐标使用

**`(0.30, ...)` 坐标来源详解：**
```
goal_distance = 0.3  # 蓝图配置，表示停在距目标 0.3m 处
z_cam = goal_distance = 0.3  # 计算相机坐标系中的目标 z 坐标
goal_cam = (z_cam, -x_cam, -y_cam) = (0.30, ...)  # 转换到 ROS 坐标系

# TF 变换失败时的 fallback（BUG！）
goal_world = goal_cam  # 相机相对坐标被当作全局坐标！
```

结果：目标 `(0.30, ...)` 直接作为全局坐标，机器人导航到原点附近。

**修复 1 — TF 树构建**：`connection.py` 强制设置 `frame_id="odom"`：
```python
odom_transform = Transform(
    translation=odom.position,
    rotation=odom.orientation,
    frame_id="odom",  # 强制设置，确保 TF 树正确构建
    child_frame_id="base_link",
    ts=odom.ts,
)
```

**修复 2 — 移除危险 fallback**：TF 变换失败时不再发布目标：
```python
if goal_world is None:
    logger.error("TF transform failed, cannot navigate")
    return  # 不发布目标，而不是错误地使用相机坐标
```

**验证日志：**
```
Transformed (x, y, z) from camera_optical to (X, Y, Z) in odom
BBox center: (...) → Goal pose: (X, Y, Z) in frame 'odom'
```

**相关文件：**
- `dimos/robot/unitree/go2/connection.py:274-310` - TF 树构建
- `dimos/navigation/bbox_navigation.py` - TF 查询和坐标变换

### 导航目标位置漂移（ObjectTracker2D 持续更新）

**症状：**
- TF 变换已成功，但机器人导航到错误位置（距离检测位置 1m+）
- 日志显示目标位置在短时间内多次更新
- 最终目标与最初检测位置相差较大

**根因：**
`ObjectTracker2D` 以 ~4Hz 持续追踪目标，每次追踪结果都触发 `BBoxNavigationModule` 发布新导航目标。机器人在移动/转向时，bbox 位置变化导致目标位置漂移。

**修复 — One-shot 模式**：每次追踪只发布一个目标：
```python
# BBoxNavigationModule.Config
one_shot: bool = True  # 每次追踪只发布一个目标
tracking_reset_timeout: float = 2.0  # 追踪停止后自动重置

# _on_detection() 中
if one_shot and _goal_published:
    return  # 不再发布新目标

# 自动重置：超过 2s 无新 detection，重置状态允许下一次追踪
if time_since_last > tracking_reset_timeout:
    _goal_published = False
```

**验证日志：**
```
BBox center: (...) → Goal pose: (X, Y, Z) in frame 'odom'  # 只出现一次
Auto-reset: No detection for 2.1s, resetting for next tracking session  # 追踪停止后重置
```

**相关文件：**
- `dimos/navigation/bbox_navigation.py:36-52` - One-shot 配置和逻辑

### 导航前调整视角（已移除 — 2026-05-15）

**⚠️ Centering 逻辑已移除。** 原因：添加 centering 后导致目标频繁丢失，反而降低成功率。

**根因分析：** 流程设计过于复杂是 navigate_to_object 失败的主要原因。每个中间步骤（等待稳定→重新检测→居中→多次VLM检测）都可能失败，累积失败率高。不是算法精度问题，不是检测速度问题，而是**中间步骤太多**。

**简化后的流程（当前实现）：**
```
检测到目标 bbox (from look_out_for)
    ↓
navigate_to_object():
  ├── cancel_goal() + Twist.zero()
  ├── 直接用 initial_bbox (不重新检测)
  └── 启动追踪 _object_tracking.track(bbox)
    ↓
ObjectTracker2D (CSRT):
  └── 持续追踪，输出 bbox
    ↓
BBoxNavigationModule:
  ├── TF 变换 camera_link → odom
  └── 发布导航目标 (one_shot=True)
    ↓
导航完成
```

**关键原则：检测到目标 → 停止探索 → 直接导航。不要添加中间步骤。**

### navigate_to_object 失败的根因分析（2026-05-15）

**问题不是：**
- ❌ 算法精度问题（VLM 检测工作正常）
- ❌ 检测速度问题（0.8s 周期，~0.24s 延迟，对静态物体足够）
- ❌ TF 变换问题（已修复）

**真正的问题：流程设计过于复杂**

原流程有多个中间步骤，每个都可能失败：
```
检测 → 验证(面积) → 停止 → 等待0.5s → 重新检测 → 居中 → 多次检测 → 追踪 → 导航
         ↓             ↓           ↓          ↓         ↓
       被拒绝         惯性移动    目标移出视野  检测失败    丢失
```

**关键教训：核心任务是"检测到目标 → 停止探索 → 导航到目标"，不要添加过多中间步骤。**

每个中间步骤（等待稳定、重新检测、居中）都增加失败风险。简化后的流程直接使用 initial_bbox 启动追踪，成功率高得多。

### 小物体（瓶子、杯子等）检测被过滤

**症状（已修复 2026-05-14）：**
- 日志中出现 `FOLLOW_CANDIDATE_REJECTED: area_ratio 0.0001 < threshold 0.01`
- 瓶子、杯子等小物体检测成功但导航未触发

**原因：** 原 `_follow_continuation_min_area_ratio = 0.01`（1%）阈值对小物体太高

**修复（2026-05-15 更新）：**
- `_navigate_to_object_min_area_ratio = 0.0015`（0.15%）专门用于静态物体导航
- 新增 `_navigate_to_object_edge_margin_ratio = 0.03`（3%）检测边缘误检
- `follow_person` 保持较高阈值（1%）避免人形误检

**⚠️ 阈值仍需调整（2026-05-15）：**
当前 0.15% 仍然高于真实小目标面积。日志数据显示：
- [568,279,607,302] → 38x23 → 0.096% → REJECTED
- [279,284,300,302] → 20x19 → 0.042% → REJECTED
- [1101,284,1119,324] → 18x40 → 0.078% → REJECTED

建议调整为 0.0003 (0.03%) 或 0.0004 (0.04%)，确保小目标不被误拒。

**阈值变更历史：**
- 初版：0.01（1%）- 对小物体太高
- 2026-05-14：0.0005（0.05%）- 允许小物体通过
- 2026-05-15：0.002（0.2%）- 过滤 VLM 小面积误检

**验证日志：**
```
FOLLOW_CANDIDATE_ACCEPTED: area_ratio=0.0076, min_threshold=0.002
FOLLOW_CANDIDATE_REJECTED: area_ratio=0.0018, reject_small_area=True
```

### VLM 误检分析与调试

**症状：**
- VLM 返回检测 bbox 但实际画面中无目标
- 通常是小面积检测（area_ratio < 0.2%）或在图像边缘

**调试方法：**
检测图像自动保存到 `~/.local/state/dimos/detection_debug/`

```
文件名格式：
{时间戳}-{目标名称}-{状态}-{面积占比}.jpg

示例：
20260515-102200-绿色瓶子-rejected-0.2pct.jpg   # 被拒绝
20260515-102205-绿色瓶子-accepted-0.5pct.jpg    # 被接受
```

**图像标注：**
- 绿色框 = 接受（valid detection）
- 红色框 = 拒绝（rejected detection）
- 图像上方标注目标名称、状态、面积比例

**常见误检特征：**
1. 小面积 bbox（area_ratio < 0.2%）
2. bbox 位于图像边缘（left_edge 或 right_edge）
3. VLM confidence 固定为 1.0（不可靠）

**VLM 检测时序分析：**
- 检测周期：0.8s（由 `.env` 的 `PERCEIVE_PERIOD=0.8` 控制）
- VLM 查询耗时：平均 0.24s，范围 0.18s-0.57s
- 检测频率：跟踪人和导航静态物体使用相同周期

**为什么不用二次验证：**
- 二次验证会增加 ~0.24s 延迟
- 期间机器人移动 ~1.3cm，可能导致目标位置变化
- 当前方案：提高面积阈值 + 边缘检测，零延迟过滤误检

### 探索停止后仍发布最后一个目标（竞争条件，已修复 2026-05-14）

**症状：**
- 检测到目标后，机器人没有立即导航到目标
- 而是先移动到其他位置，然后才停止
- 日志显示探索在检测到目标后才停止，但最后一个前沿目标仍在执行

**根因：** 
探索循环在 `end_exploration()` 调用后，循环还没退出就发布了最后一个目标：
- `12:20:15.943` - 检测到可乐
- `12:20:16.287` - 探索停止
- `12:20:16.288` - **探索发布了最后一个目标！**（竞争条件）

**修复：** `wavefront_frontier_goal_selector.py:789`
- 在发布目标前检查 `stop_event` 和 `exploration_active`
- 已停止则立即退出，不发布目标

**验证日志：**
```
Exploration stopped before publishing goal
```

### stop_exploration() 发布停止目标导致导航竞争（已修复 2026-05-14）

**症状：**
- 检测到静态物体（如可乐瓶）后，机器人没有正确导航到目标位置
- 导航目标位置偏差约 0.3m，机器人移动到错误位置
- 日志显示 `stop_exploration()` 后仍有目标被执行

**根因：**
`stop_exploration()` 在停止探索时会发布当前位置作为导航目标（意图是让机器人停止移动）：
```
12:28:00.262 Got new goal ← 停止目标被接收
12:28:00.262 Cancelling goal ← 尝试取消
12:28:00.263 Close enough to goal. Accepting as arrived. ← 目标仍被执行！
12:28:00.264 Travelling to goal 0.298m away ← 机器人移动了 0.3m
```
这导致了竞争条件：目标被 global_planner 接收后，即使 `cancel_goal()` 也无法阻止执行。

**修复：** `wavefront_frontier_goal_selector.py:750`
- 移除 `stop_exploration()` 中发布当前位置目标的代码
- 改为由调用者负责停止机器人（通过 `cancel_goal()`）
- 添加注释说明原因

**验证日志：**
- `stop_exploration()` 后不应看到 `Got new goal` 来自探索模块
- 导航目标应正确指向检测到的物体位置

### BBoxNavigation 导航目标频繁更新导致无法执行

**症状（已修复 2026-05-14）：**
- 目标检测成功但机器人不移动
- 日志显示导航目标每 ~240ms 更新一次
- 导航状态在 `following_path` 和 `idle` 之间频繁切换

**原因：** ObjectTracker2D 以 ~4Hz 更新 bbox，每次更新都触发新导航目标，导致前一目标被取消

**修复：**
- BBoxNavigationModule 新增 `position_threshold: float = 0.2` 参数
- 只有当目标位置变化超过 0.2m 才发布新导航目标
- 导航状态稳定，机器人可以正常执行导航

**验证日志：**
```
Goal position change (0.029m) below threshold (0.2m), skipping update
Tracker update: succeeded=True, bbox=(877.0, 167.0, 74.0, 69.0)
```

## 执行步骤

### 1. 启动目标检测（推荐使用 auto_explore）

**检测+探索+自动导航模式（推荐，一个命令完成）：**

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["water bottle", "水瓶"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "water bottle"}}}'
```

返回：`Started looking for [...]. Continuation logic armed. Exploration started.`

**纯检测模式（机器人不移动）：**
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"]}'
```

**检测+自动跟踪模式：**
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"], "auto_explore": true, "then": {"name": "follow_person", "arguments": {"query": "person"}}}'
```

参数说明：
- `description_of_things`: 要寻找的目标列表
- `auto_explore`: （推荐）自动启动探索，机器人移动寻找目标
- `then`: （可选）当 VLM 视觉模型确认检测到目标后自动触发的后续动作

### 2. 监控执行过程

```bash
dimos log -f
```

关键日志标志：
- `Started looking for ["person"]. Continuation logic armed.` — 检测启动成功
- `POLLING_MATCH: Verified ["person"] at ...` — 视觉模型确认发现目标
- `MCP tool call tool=follow_person` — 开始跟踪
- `Cancelled navigation goal during person follow  reason=person-follow handoff start state=...` — handoff 第一次 cancel 残留导航
- `Cancelled navigation goal during person follow  reason=person-follow handoff post-settle state=...` — handoff 第二次 cancel 完成
- `Refined person-follow handoff on fresh frame ... attempt=1` — 新帧重定位成功
- `EdgeTAM initialized with N detections` — 追踪器初始化成功
- `Found the person. Starting to follow.` — 跟踪成功启动
- `No path found to the goal.` 连续出现 → exploration 已死，需 `dimos restart`

### 4. 任务结束后清理

```bash
dimos mcp call end_exploration
dimos mcp call stop_looking_out
dimos mcp call stop_following
```

**⚠️ 重要：** `stop_looking_out` 只停止检测循环，**不会停止已由 `then` continuation 触发的 `follow_person`**。如果 `look_out_for` 已经触发跟踪，必须同时调用 `stop_following` 才能完全停止。

**正确清理顺序：**
1. `stop_looking_out` - 停止检测
2. `stop_following` - 停止跟踪（如果已触发）
3. `end_exploration` - 停止探索

## 已修复的关键问题

### 0. `navigate_to_object` 完成后检测循环不停止（已修复 2026-05-14）

**症状：**
- 导航完成后，VLM 检测循环仍在运行，不停刷日志
- 日志显示 `POLLING_QUERY`、`VLM_DETECTION_QUERY` 持续出现
- `FOLLOW_CANDIDATE_REJECTED` 重复出现

**根因：**
`_handle_match` 只在 `then=None` 时才停止检测循环。对于 `navigate_to_object`（静态物体导航），导航完成后应该停止检测。

**修复：**
- `navigate_to_object` 触发后停止检测循环（一次性任务）
- `follow_person` 保持检测循环运行（需要持续更新 bbox）

**验证日志：**
```
STATIC_NAVIGATION_COMPLETE: Stopping detection loop
```

### 1. `navigate_to_object` 阻塞检测循环（已修复 2026-05-14）

**症状：**
- `look_out_for` + `then: navigate_to_object` 模式下，检测循环在发现目标后停止
- 日志中没有后续的 `POLLING_MATCH`，检测循环不再运行
- `auto_follow=False` 但 `dispatch_continuation` 后无后续检测

**根因：**
`dispatch_continuation` 在 `perceive_loop_skill.py` 的 `_polling_match_loop` 中是**同步调用**。原 `navigate_to_object` 会阻塞等待导航完成（最多 30 秒），期间检测循环完全停止。

**修复：** `navigate_to_object` 现在异步返回：
- 启动 `ObjectTracker2D` 后立即返回，不等待导航完成
- 导航在后台由 `BBoxNavigationModule` 执行
- 检测循环可以立即继续下一次检测

**相关修改：**
1. `_FOLLOW_CONTINUATION_TOOLS` 列表添加 `"navigate_to_object"`（`mcp_client.py:44`）
2. `_augment_follow_continuation_args` 自动注入 `initial_bbox` 和 `initial_image`
3. `navigate_to_object` 函数签名添加可选参数

**验证日志：**
```
CONTINUATION_TRIGGER: navigate_to_object called with bbox=[...]
auto_follow=False
# 随后应继续看到 POLLING_MATCH 日志（检测循环继续）
```

### 1. Agent-MCP `follow_person` 重复调用竞争（已修复）

**修复文件 1：** `dimos/agents/mcp/mcp_client.py:393`
- Agent 现在会对刚由 `look_out_for` continuation 自动执行过的 `follow_person` / `follow_person_with_monocd` / `detect_person_with_monocd` 做 **5 秒短时抑制**
- 按工具名 + query 去重；如果 Agent 在窗口期内又想调同一个跟随工具，会直接返回跳过，不再把第二个 MCP 请求打到 `McpServer`

**修复文件 2：** `dimos/agents/skills/perceive_loop_skill.py:207`
- `look_out_for` 命中且 `then` 有效时，**只走 `dispatch_continuation(...)`**，不再先额外塞一条原始 "SUCCESS: Target verified" 消息给 Agent
- 这样一次命中不会再同时触发"原始检测消息 + continuation 已执行消息"两路刺激
- 标注后的证据图仍然被放进 continuation context 里

**结果：** `follow_person` 现在可以安全地直接作为 `look_out_for` 的 `then` 使用，不再出现 Agent 和 continuation 同时调用导致 EdgeTAM 被初始化两次的问题。

### 2. EdgeTAM 初始化帧错位导致启动即丢失（已修复）

**修复文件：** `dimos/agents/skills/person_follow.py:253` 和 `:287`

当 `follow_person` 收到 `look_out_for` 的 continuation handoff（带 `initial_bbox`）时，现在的逻辑是：
1. 先停掉 patrol / exploration
2. **立即 `cancel_goal()` 一次**（清掉导航队列中残留的 exploration/patrol goal）
3. 先发 `cmd_vel = 0`
4. **等待 3 个更新后的相机帧**（让机器人完全停稳，画面稳定）
5. **再 `cancel_goal()` 一次**（专门清掉 stop_exploration/stop_patrol 退出时延迟送到的"当前位置 goal"）
6. 在新帧上用 `query` 重新定位目标，**最多尝试 5 次**
7. 如果重定位成功，用**新帧 + 新 bbox** 初始化 EdgeTAM
8. 如果重定位失败，回退到原始 detection frame 初始化 EdgeTAM，但**不会立刻进入正式跟随** — 而是先进行 **tracker warm-up**：在最多 2 个 live frame 上以零速度预滚动，只有 tracker 在当前画面里重新锁住目标才继续；否则直接失败返回，避免"初始化成功但 1 秒后就丢失"的坏路径

典型日志（成功路径）：
```
Cancelled navigation goal during person follow  reason=person-follow handoff start state=following_path
Cancelled navigation goal during person follow  reason=person-follow handoff post-settle state=idle
Refined person-follow handoff on fresh frame ... attempt=1
EdgeTAM initialized with 1 detections
```

如果 fresh-frame 重定位失败，可能看到：
```
Failed to reacquire person for fresh handoff frame; falling back to original frame
EdgeTAM initialized with 1 detections
```
此时 warm-up 会在后台运行；若 warm-up 未锁住目标，`follow_person` 会直接返回失败，而不是进入 1 秒就丢的坏路径。

这避免了"在旧检测帧上 init EdgeTAM，但第一批 tracking frame 已经是机器人继续移动后的新视角"这个错位，同时彻底消除了残留导航 goal 对 cmd_vel 的抢占。

### 3. 跟随期间导航残留抢占速度控制（已修复）

**修复文件：** `dimos/agents/skills/person_follow.py`

`ReplanningAStarPlanner` 和 `follow_person` 都往同一个 `/cmd_vel` 发控制。如果 exploration/patrol 残留的 goal 让 `local_planner` 保持非 IDLE 状态，它会和 visual servoing 抢速度输出。

修复方式：
- 在 `follow_person` 的 `_follow_loop` 中**周期性检查导航状态**
- 只要检测到 `local_planner` 不是 `idle`，就立即 `cancel_goal()`
- 带有限频保护，避免高频刷屏

**回归测试：** `dimos/agents/skills/test_person_follow.py:142`
- 验证 handoff 期间会做两次 navigation cancel
- 验证导航空闲时不会误 cancel
- 验证导航活跃时会 cancel，并带限频

### 4. Agent 历史消息溢出导致上下文超限（已修复）

**症状：**
- Agent 日志中出现 `Error code: 400 - maximum context length is 4096 tokens`
- Agent 无法处理新消息，整个系统卡住

**根因：** `McpClient._history` 列表无限增长，超过模型上下文 token 限制

**修复：** `dimos/agents/mcp/mcp_client.py`
- 新增 `McpClientConfig.max_history_messages=50` 配置参数
- 新增 `_prune_history_for_model()` 方法，保留系统消息 + 最近消息
- 新增 `_drop_image_content_blocks()` 函数，可移除消息中的图像数据块（减少 token 数）
- 在 `_process_message()` 中自动调用截断逻辑

**配置：** 可通过环境变量 `DIMOS_MAX_HISTORY_MESSAGES` 或 blueprint 配置覆盖默认值

## 仍然存在的已知限制

### `look_out_for` 不监控探索状态（架构问题，待修复）

**症状：**
- `look_out_for` + `auto_explore=true` 模式下，探索在发布 2 个目标后停止
- 日志显示 `Exploration complete after 2 goals and 10 consecutive failures finding new frontiers`
- 但 `POLLING_QUERY` 仍在继续，机器人停在原地空转检测
- VLM 一直返回空检测结果 `raw_response: []`

**根因：**
`look_out_for` 和探索模块（`WavefrontFrontierExplorer`）是**独立运行**的：
1. `look_out_for` 通过 `ExplorationModuleSpec.begin_exploration()` 启动探索
2. 探索模块有自己的结束逻辑：`goals_published >= 2 and consecutive_failures >= 10`
3. `look_out_for` 的感知循环**不监控探索状态**，不知道探索已结束
4. 结果：机器人停止移动，但检测循环继续空转

**架构图：**
```
look_out_for                    WavefrontFrontierExplorer
    │                                    │
    ├─ 启动探索 ─────────────────────→  开始探索
    │                                    │
    ├─ 每 0.8s 检测目标                  ├─ 发布 frontier 目标
    │                                    │
    │                                    ├─ 找不到新 frontier
    │                                    │
    │                                    ├─ 连续失败 10 次
    │                                    │
    │                                    └─ exploration_active=False
    │                                           │
    ├─ 继续检测（不知道探索已结束）←───────────────┘
    │
    └─ 原地空转...
```

**修复方案（待实现）：**
在 `_perception_loop` 中添加探索状态检查：
```python
# 如果探索已结束但目标未找到，重新启动探索
if self._auto_started_exploration and not self._exploration_module_spec.is_exploration_active():
    logger.info("Exploration ended without finding target, restarting exploration...")
    self._exploration_module_spec.begin_exploration()
```

**临时解决方案：**
用户需手动监控日志，发现 `Exploration complete` 后重启探索：
```bash
dimos mcp call begin_exploration
```

**相关文件：**
- `dimos/perception/perceive_loop_skill.py:174-179` - `look_out_for` 启动探索的代码
- `dimos/navigation/frontier_exploration/wavefront_frontier_goal_selector.py:845-850` - 探索结束逻辑
- `dimos/navigation/frontier_exploration/exploration_module_spec.py` - 探索模块接口

### `follow_person` 执行时不停后退（已修复）

**根因：** `VisualServoing2D` 默认 `_assumed_object_width = 0.45m`。对于手持杯子的人，手臂/杯子导致 bbox 过宽，距离被低估，触发持续后退。

**结构性修复（推荐参数）：** 修改 `dimos/navigation/visual_servoing/visual_servoing_2d.py:30` 附近：

```python
# 温和宽框修正 — 避免手臂/杯子导致距离被严重低估
_max_person_aspect_ratio: float = 0.65
_max_width_correction_factor: float = 1.2

# hold deadband 收窄 — 避免在合适距离过早停住
_hold_deadband: float = 0.1  # 原为 0.2

# creep 前进 — 当目标略远于理想距离时，给很小正向速度继续贴上去
# （在 compute_twist 中实现：estimated_distance 处于 target_distance 附近时输出小的正 linear_x）
```

同时保留：**只有 estimated_distance 明显小于 _min_distance 时才倒退**，彻底杜绝了长期后退。

**`follow_person` 专属"紧跟"参数（不污染全局默认值）：**

`dimos/agents/skills/person_follow.py:47` 的 `Config` 新增了 4 个参数，默认更紧：

```python
follow_target_distance_2d: float = 1.0
follow_min_distance_2d: float = 0.55
follow_distance_deadband_2d: float = 0.05
follow_min_forward_speed_2d: float = 0.12
```

在 `person_follow.py:109` 初始化 `VisualServoing2D` 时，通过构造参数直接覆盖全局默认值。这样 `follow_person` 会比其他 2D 伺服任务更积极贴近目标，同时保持宽框修正和防倒退保护。

**验证效果（模拟器）：**
- 跟踪持续时间从 18 秒提升到 **38 秒以上**
- 未出现长期倒退
- 机器人主动从 ~0.14m 靠近并稳定在 ~0.7–1.0m 范围持续跟随

**回归测试：** `dimos/navigation/visual_servoing/test_visual_servoing_2d.py:28` 覆盖 5 个场景：
1. 宽框距离修正
2. 宽框不误触发倒退
3. 极近时仍然会倒退
4. 略远时会小步跟进（creep）
5. 明显远时会正常前进

修改后需要 `dimos restart`。

### `follow_person` 的 EdgeTAM/SAM2 环境问题

1. **首次使用需要联网下载权重**（`repvit_m1.dist_in1k` 等）
2. **偶发 `No available kernel. Aborting execution.`** — SAM2 transformer 的 scaled_dot_product_attention 在当前 PyTorch/CUDA 环境下可能找不到可用 kernel

如果首次调用成功，后续通常可稳定运行。

### 跟踪时保持目标在画面中心（垂直居中）

**现象：** `follow_person` 只调整 yaw（水平转向），不调整 pitch（俯仰），目标容易跑到画面上下边缘而丢失。

**修复：** 修改 `dimos/navigation/visual_servoing/visual_servoing_2d.py`，让 `compute_twist` 同时输出 `angular.y`（pitch）速度；并在 `dimos/agents/skills/person_follow.py` 的 `_follow_loop` 中去掉 `latest_image.width` 参数。
详细修改步骤见历史版本 skill 文档。

### 墙角/墙角处跟踪卡住或丢失（已部分修复）

**现象：** 当目标位于侧前方（如墙角）时，机器人会一边转向一边继续前冲，容易撞到墙壁或障碍物，随后被 `local_planner` 的障碍物检测阻止，导致长时间停滞并最终丢失目标。

**修复 1 — "先转后走"门限：** `visual_servoing_2d.py:153`
- 当目标横向偏离画面中心过大时，直接把 `linear_x` 压到 0，优先用 `angular_z` 将目标转回画面中间，再恢复前进
- 这避免了"目标在侧前方，机器人还继续硬顶前进"的撞墙行为

**修复 2 — 加快转向收敛：**
- 如果角速度增益不足，机器人可能在原地"零速慢转"长达 10 秒以上，目标移动后反而更容易跟丢
- 建议同时提高"先转后走"阶段的 `angular_z` 增益，或采用渐进释放策略：`linear_x` 不完全为 0，而是随偏移量衰减（`scale = max(0, 1 - |offset|/threshold)`），实现边转边走

**修复 3 — handoff 稳定期延长：** `person_follow.py`
- settle frame 从 2 帧提升到 **3 帧**
- fresh-frame 重定位尝试从 3 次提升到 **5 次**
- 即使回退旧帧，也会执行 **tracker warm-up**（最多 2 个 live frame 零速预滚动），确认 tracker 重新锁住才进入正式跟随

**典型失败标志（仍需调参）：**
- 机器人在墙角附近停滞 10 秒以上，位置几乎不变
- 随后 `lost track of the person`
- 这说明"先转后走"的转向收敛速度还是太慢，需要进一步提高角速度或放宽 linear_x 的压制门限

### 模拟环境中 exploration 可能走入死角

在 simulation 中 `begin_exploration` 偶尔会因 `No path found to the goal` 连续失败而自动停止，导致机器人原地不动，且 `look_out_for` 永远无法检测到目标。

**判断标志：**
- 日志中持续出现 `wavefront_frontier_goal_selector.py Goal timeout`
- 随后 `Stopped autonomous frontier exploration`
- 接着大量 `global_planner.py No path found to the goal`
- 长时间没有 `POLLING_MATCH` 或 `follow_person` 触发

**处理方案：**
1. **优先：** 直接 `dimos restart` 或 `dimos stop && dimos --simulation run unitree-go2-agentic` 重启整个实例（仿真环境会重新初始化，通常能恢复）
2. 如果机器人只是短暂卡住，可以尝试清理后重新启动 `begin_exploration` + `look_out_for`

**⚠️ 重要：Run ID 会变化**
- `dimos restart` 或 `dimos stop && dimos run ...` 后，Run ID 会改变，日志路径也随之改变（`~/.local/state/dimos/logs/<run-id>/main.jsonl`）
- 任何硬编码了 Run ID 或日志路径的监控脚本都会失效
- 使用 `scripts/log_monitor_loop.py` 可自动从 `dimos status` 重新解析当前 Run ID

## Agent-Driven 监控循环（首选方式）

**推荐使用 agent 直接监控**。架构：后台日志监听器（传感器）+ Agent 决策（大脑）。

```
┌─────────────────────┐     每 3 秒      ┌─────────────────────────┐
│  log_watcher.py     │ ─── 写状态 ───→  │  /tmp/dimos_state.json  │
│  (后台轻量进程)      │                  │  {state, cycle, ...}    │
│  只读日志，不做决策   │                  └──────────┬──────────────┘
└─────────────────────┘                             │
                                            agent 读文件 (毫秒级)
                                                    │
                                            ┌───────▼────────┐
                                            │  Agent 决策     │
                                            │  清理/重启/等待  │
                                            └────────────────┘
```

**优势：**
- 轮询间隔 3-5 秒（读状态文件只需毫秒）
- Agent 两次检查之间可以并行执行其他任务
- 决策权完全在 agent，能灵活应对意外情况
- 后台脚本只负责读日志，不调用 MCP，不会干扰机器人

### 第一步：启动后台监听器

```bash
# 后台运行，自动检测日志路径，3 秒轮询
python3 -u ~/.hermes/skills/robotics/dimos-explore-find-navigate/scripts/log_watcher.py &

# 或手动指定日志路径和轮询间隔
python3 -u ~/.hermes/skills/robotics/dimos-explore-find-navigate/scripts/log_watcher.py \
    --log ~/.local/state/dimos/logs/<run-id>/main.jsonl \
    --state /tmp/dimos_state.json \
    --interval 3
```

### 第二步：初始化机器人

**⚠️ 关键：必须在一个 execute_code 块中同时启动探索和检测，否则会串行执行导致延迟。**

**纯检测模式：**
```python
import subprocess, json

dimos = "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos"

# 清理旧状态
for cmd in ["stop_following", "stop_looking_out", "stop_navigation", "end_exploration"]:
    subprocess.run([dimos, "mcp", "call", cmd], capture_output=True)

# 启动探索（可选）
subprocess.run([dimos, "mcp", "call", "begin_exploration"], capture_output=True)

# 纯检测，不自动跟踪
subprocess.run([dimos, "mcp", "call", "look_out_for", "--json-args",
    json.dumps({"description_of_things": ["person"]})],
    capture_output=True)

print("检测已启动（纯检测模式）")
```

**检测+自动跟踪模式：**
```python
import subprocess, json

dimos = "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos"

# 清理旧状态
for cmd in ["stop_following", "stop_looking_out", "stop_navigation", "end_exploration"]:
    subprocess.run([dimos, "mcp", "call", cmd], capture_output=True)

# 标记起点
subprocess.run([dimos, "mcp", "call", "tag_location", "--json-args", '{"location_name": "home"}'])

# 同时启动探索和检测+跟踪
subprocess.run([dimos, "mcp", "call", "begin_exploration"], capture_output=True)
subprocess.run([dimos, "mcp", "call", "look_out_for", "--json-args",
    json.dumps({"description_of_things": ["person holding a green cup", "human with green cup", "green cup"],
                "then": {"name": "follow_person", "arguments": {"query": "person holding a green cup"}}})],
    capture_output=True)

print("探索和检测+跟踪已并行启动")
```

### 第三步：Agent 轮询（每轮 execute_code 毫秒级完成）

```python
# Agent 每轮执行的代码
import json
with open("/tmp/dimos_state.json") as f:
    s = json.load(f)
print(f"state={s['state']} cycle={s['cycle']} age={s['age_seconds']}s")
print(f"last_event: {s['last_event']}")
print(f"no_path: {s['summary']['no_path_count']}")
```

### 状态机与 Agent 执行命令

```
EXPLORING ──FOUND──→ FOLLOWING ──LOST──→ LOST ──等5秒──→ 清理重启 → EXPLORING
    │
    └─EXPLORATION_STOPPED → 只重启 begin_exploration（look_out_for 还在运行）
    └─no_path_count > 8 → 清理全部，重启 explore + look_out_for
```

**状态转换由 Agent 主动执行 MCP 命令驱动。** watcher 只报告状态，不做决策、不发命令。

**Agent 决策表（每个状态对应的具体命令）：**

| 状态文件 state | Agent 执行的命令 |
|---------------|-----------------|
| `EXPLORING` | 无需动作，继续下一轮轮询 |
| `FOLLOWING` | 无需动作，继续下一轮轮询 |
| `EXPLORATION_STOPPED` | `dimos mcp call begin_exploration`（look_out_for 还在运行，不用重启） |
| `LOST` | 等 5 秒后执行清理重启序列（见下方） |
| `no_path_count > 8` | 执行清理重启序列（见下方） |
| 文件不存在 | `python3 -u log_watcher.py &` 重新启动监听器 |

**清理重启序列（LOST 或 no_path_count > 8 时执行）：**

```bash
# 1. 停止所有活动
dimos mcp call stop_following
dimos mcp call stop_looking_out
dimos mcp call stop_navigation
dimos mcp call end_exploration

# 2. 等待 2 秒确保清理完成

# 3. 重启探索 + 检测
# 纯检测模式：
dimos mcp call begin_exploration
dimos mcp call look_out_for --json-args '{"description_of_things": ["<目标描述>"]}'

# 或 检测+跟踪模式：
dimos mcp call begin_exploration
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["<目标描述>"],
  "then": {"name": "follow_person", "arguments": {"query": "<目标描述>"}}
}'
```

**Agent 轮询模板（每次 execute_code 调用）：**

```python
import json, os

state_file = "/tmp/dimos_state.json"
if not os.path.exists(state_file):
    print("ACTION: restart_watcher")
else:
    s = json.load(open(state_file))
    state = s["state"]
    no_path = s["summary"]["no_path_count"]

    if state == "FOLLOWING":
        print(f"OK: following (cycle {s['cycle']})")
    elif state == "EXPLORING":
        if no_path > 8:
            print("ACTION: full_restart  # no_path_count > 8")
        else:
            print(f"OK: exploring (no_path={no_path})")
    elif state == "EXPLORATION_STOPPED":
        print("ACTION: restart_exploration_only  # look_out_for still running")
    elif state == "LOST":
        print("ACTION: wait_5s_then_full_restart")
```

Agent 读取 ACTION 后，在下一轮 execute_code 中执行对应的 MCP 命令。

### 并行执行

Agent 在两次状态检查之间（3-5 秒间隔）可以同时：
- 回应用户消息
- 执行导航命令（如 `navigate_with_text` 返回原点）
- 查看其他日志或调试信息
- 启动 `delegate_task` 处理其他任务

### 监控注意事项

1. **`look_out_for` 在 exploration 停止后仍然持续运行**。当 exploration 因 "No information gain" 停止时，只需重启 `begin_exploration`，不需要重新启动 `look_out_for`。可通过调用 `look_out_for --json-args '{"description_of_things": ["test"]}'` 验证（返回 "Already looking for something else" 表示仍在运行）。

2. **Exploration 频繁停止**：模拟环境中地图被充分探索后，frontier exploration 会在几分钟内停止。agent 必须能检测到并重启。

3. **日志路径可能变化**：`dimos restart` 后 Run ID 改变，日志路径也变。`log_watcher.py` 会自动检测并切换。

4. **检测与跟踪独立运行**。即使 `follow_person` 因依赖问题（如 turbojpeg 缺失）失败，`look_out_for` 的检测循环仍然继续运行。下次检测到目标时会再次尝试启动跟踪。

5. **目标检测状态可在日志中确认**。即使跟踪未启动，也可通过 `POLLING_MATCH` 日志确认目标已被视觉模型发现，这有助于区分是检测问题还是跟踪问题。

6. **⚠️ 禁止分两次 execute_code 调用 `begin_exploration` 和 `look_out_for`**。Agent 必须在单次 execute_code 中同时调用两者，否则会串行执行导致 30+ 秒延迟。每次调用新工具前，先检查当前状态，避免重复启动已运行的服务。

## 脚本目录（scripts/）

所有需要脚本辅助的场景，脚本统一放在 skill 的 `scripts/` 目录下，**不要在 /tmp 创建临时脚本**。

**当前脚本：**

| 文件 | 功能 | 使用场景 |
|------|------|----------|
| `log_watcher.py` | 后台日志监听器，写状态文件 | Agent-driven 监控的传感器组件（首选） |
| `log_monitor_loop.py` | 完整的自动循环监控 | 无人值守时独立运行（备选） |

### log_watcher.py — 日志监听器（首选方案配套）

后台轻量进程，持续 tail 日志文件，每 3 秒将状态写入 `/tmp/dimos_state.json`。**不调用任何 MCP 命令，不做决策**，只充当 agent 的"传感器"。

```bash
# 启动
python3 -u ~/.hermes/skills/robotics/dimos-explore-find-navigate/scripts/log_watcher.py

# 参数：
#   --log      日志路径（默认自动检测）
#   --state    状态文件路径（默认 /tmp/dimos_state.json）
#   --interval 轮询间隔秒数（默认 3）
```

输出状态文件格式：
```json
{
  "state": "EXPLORING",
  "last_event": "...",
  "last_event_time": "HH:MM:SS",
  "cycle": 1,
  "log_ts": "ISO timestamp",
  "age_seconds": 5.2,
  "log_path": "/path/to/main.jsonl",
  "summary": {"no_path_count": 0}
}
```

### log_monitor_loop.py — 自动循环监控脚本（备选）

当 agent 无法持续监控（如需要长时间无人值守运行）时，可用此脚本替代。自带完整状态机和自动重启逻辑。

```bash
python3 -u ~/.hermes/skills/robotics/dimos-explore-find-navigate/scripts/log_monitor_loop.py \
    --target "person holding a green cup" \
    --desc "person holding a green cup" "human with green cup" "green cup"

# 参数：
#   --target   follow_person 的查询字符串
#   --desc     look_out_for 的检测描述列表
#   --check    日志轮询间隔（秒，默认 8）
#   --stale    无进展超时（秒，默认 90）
```

**⚠️ 注意：** 必须用 `python3 -u` 运行（无缓冲模式），否则后台运行时 stdout 不会实时输出。

## 位置标记与返回原点

DimOS 没有绝对坐标导航工具（如 `navigate_to(x,y,z)`），但提供以下语义地图导航工具：

### 标记当前位置

```bash
dimos mcp call tag_location --json-args '{"location_name": "origin"}'
```

在机器人处于某个位置时调用，将该位置与名称关联到语义地图（spatial memory）。

### 导航到已标记位置

```bash
dimos mcp call navigate_with_text --json-args '{"query": "origin"}'
```

通过自然语言查询语义地图，匹配到已标记的位置后自动规划路径导航过去。

### 实用技巧

- **模拟环境**：起点通常已在语义地图中，可以直接 `navigate_with_text("starting position origin")` 返回，无需事先标记
- **建议在任务开始时先标记起点**：`tag_location("home")`，方便任务结束后返回
- `navigate_with_text` 也支持模糊查询，如 `"starting position"`、`"home base"` 等
- `stop_navigation` 可随时取消导航

## 验证标准

- [ ] `dimos mcp call begin_exploration` 返回成功
- [ ] `dimos mcp call look_out_for` 返回 "Continuation logic armed"
- [ ] 日志中出现 `POLLING_MATCH` 和 `CONTINUATION_TRIGGER`
- [ ] `follow_person` 被自动调用
- [ ] 日志中出现 `Cancelled navigation goal during person follow`（handoff 双 cancel 生效）
- [ ] 日志中出现 `Refined person-follow handoff on fresh frame`（新帧重定位成功）
- [ ] 日志中出现 `Found the person. Starting to follow.`
- [ ] **没有**出现 `Brain calling tool: follow_person`（Agent 重复调用）
- [ ] 机器人**没有**长期后退（位置坐标不应持续减小远离目标）
- [ ] 跟踪能持续 **30 秒以上**不丢失（理想情况下可持续 60 秒以上）
- [ ] 跟随期间 `local_planner` 保持 `idle`（无残留导航抢 cmd_vel）
- [ ] 未出现 `Failed to reacquire person for fresh handoff frame` 后立刻 `lost track` 的 1 秒丢目标现象
- [ ] 丢失后执行清理 → 重启 `begin_exploration` + `look_out_for` 能再次恢复循环
