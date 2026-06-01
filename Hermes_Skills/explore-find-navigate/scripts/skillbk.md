---
name: dimos-explore-find-navigate
title: DimOS 探索-发现-跟踪 Tool 链
version: 3.0.0
description: 控制 DimOS 机械狗自主探索、发现目标并自动跟踪。简化的三步流程：清理 → 启动 → 监控。
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
---

# DimOS 探索-发现-跟踪 Tool 链

## 三步流程

### 第一步：清理旧状态

停止所有正在运行的任务：

```python
import subprocess
dimos = "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos"

# 按顺序清理
for cmd in ["stop_following", "stop_looking_out", "end_exploration"]:
    subprocess.run([dimos, "mcp", "call", cmd], capture_output=True)

print("清理完成")
```

### 第二步：同时启动探索和检测

**关键：必须在一个 execute_code 块中同时调用 `begin_exploration` 和 `look_out_for`，否则会串行执行导致 30+ 秒延迟。**

```python
import subprocess, json
dimos = "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos"

# 标记起点（可选，方便后续返回）
subprocess.run([dimos, "mcp", "call", "tag_location", "--json-args", '{"location_name": "home"}'])

# 同时启动探索和检测+跟踪
subprocess.run([dimos, "mcp", "call", "begin_exploration"], capture_output=True)

subprocess.run([dimos, "mcp", "call", "look_out_for", "--json-args",
    json.dumps({
        "description_of_things": ["person"],
        "then": {"name": "follow_person", "arguments": {"query": "person"}}
    })], capture_output=True)

print("探索和检测+跟踪已并行启动")
```

### 第三步：监控状态

读取状态文件监控执行进度：

```python
import json, os

state_file = "/tmp/dimos_state.json"
if not os.path.exists(state_file):
    print("状态文件不存在，可能需要启动 log_watcher.py")
else:
    s = json.load(open(state_file))
    print(f"state={s['state']} cycle={s['cycle']} age={s['age_seconds']}s")
    print(f"last_event: {s['last_event']}")
```

**状态含义：**
- `EXPLORING` - 正在探索，正常
- `FOLLOWING` - 已发现目标，正在跟踪
- `EXPLORATION_STOPPED` - 探索停止但检测仍运行，需重启 `begin_exploration`
- `LOST` - 目标丢失，等5秒后执行清理重启序列

## MCP 命令格式

### 检测并自动跟踪（推荐）

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"], "then": {"name": "follow_person", "arguments": {"query": "person"}}}'
```

### 纯检测（不跟踪）

```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"]}'
```

**参数说明：**
- `description_of_things`: 目标列表数组，如 `["person", "human"]`
- `then`: 发现目标后自动触发的后续动作，必须包含 `name` 和 `arguments`

**常见错误：**

```bash
# ❌ 错误：target 不是有效参数
dimos mcp call look_out_for --json-args '{"target": "person"}'

# ❌ 错误：then 格式不完整
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"], "then": "follow_person"}'

# ✅ 正确格式
dimos mcp call look_out_for --json-args '{"description_of_things": ["person"], "then": {"name": "follow_person", "arguments": {"query": "person"}}}'
```

## 状态转换与 Agent 决策

```
EXPLORING ──FOUND──→ FOLLOWING ──LOST──→ LOST ──等5秒──→ 清理重启
    │
    └─EXPLORATION_STOPPED → 只重启 begin_exploration
```

| 状态 | Agent 动作 |
|-----|-----------|
| `EXPLORING` | 无需动作 |
| `FOLLOWING` | 无需动作 |
| `EXPLORATION_STOPPED` | `dimos mcp call begin_exploration` |
| `LOST` | 等5秒后执行清理重启 |

**清理重启序列：**

```python
import subprocess
dimos = "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos"

# 1. 停止所有
for cmd in ["stop_following", "stop_looking_out", "end_exploration"]:
    subprocess.run([dimos, "mcp", "call", cmd], capture_output=True)

# 2. 等待2秒
import time; time.sleep(2)

# 3. 重启探索+检测
subprocess.run([dimos, "mcp", "call", "begin_exploration"], capture_output=True)
subprocess.run([dimos, "mcp", "call", "look_out_for", "--json-args",
    json.dumps({"description_of_things": ["person"], "then": {"name": "follow_person", "arguments": {"query": "person"}}})],
    capture_output=True)
```

## 前置条件

1. DimOS 正在运行：`dimos --simulation run unitree-go2-agentic`
2. turbojpeg 已安装：`pip install turbojpeg`

## 后台监听器（可选）

启动状态文件写入器：

```bash
python3 -u ~/.hermes/skills/dimos-explore-find-navigate/scripts/log_watcher.py &
```

## 返回原点

```bash
# 标记当前位置
dimos mcp call tag_location --json-args '{"location_name": "home"}'

# 导航返回
dimos mcp call navigate_with_text --json-args '{"query": "home"}'
```
