---
name: piper-arm-control
description: Control Piper robotic arm for pick operations
version: 2.1.0
author: Hermes
tags: [dimos, piper, arm, grasp, pick]
triggers:
  - "抓取"
  - "拿起"
  - "拾取"
  - "抓.*瓶子"
  - "抓.*杯子"
  - "pick up"
  - "grab"
---

# Piper Arm Control

Control the Piper robotic arm to pick up objects using Contact-GraspNet.

## ⚠️ 重要：独立抓取 vs 完整链式抓取

**当用户说"寻找X并抓取"或"找X并拿起"时，不要使用独立的 `pick` 命令！**

| 用户指令 | 正确命令 |
|---------|---------|
| "寻找绿色瓶子并抓取" | `look_out_for` → `navigate_to_object` → `pick` (完整链) |
| "找水瓶并拿起" | `look_out_for` → `navigate_to_object` → `pick` (完整链) |
| "抓取瓶子" (机器人已在瓶子前) | `pick` (独立命令) |
| "拿起杯子" (机器人已在杯子前) | `pick` (独立命令) |

**判断规则：**
- 用户说了"寻找/找/搜索 + 抓取/拿起" → 完整链（explore + navigate + pick）
- 用户只说了"抓取/拿起"（没有"寻找/找"） → 独立 pick 命令
- 不确定时 → 询问用户机器人是否已在目标前

**完整链格式（寻找+抓取）：**
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

**独立 pick 格式（机器人已到位）：**
```bash
dimos mcp call pick --json-args '{"object_name": "bottle"}'
```

## Integration Status

**PiperArmSkillContainer is integrated in `unitree-go2-agentic` blueprint.**

Location: `dimos/robot/manipulators/piper/piper_arm_skill_container.py`
Blueprint: `_common_agentic.py` - `PiperArmSkillContainer.blueprint()`
Scripts: `dimos/robot/manipulators/piper/scripts/`

## Architecture

```
PGX (本地)                                   Jetson (192.168.12.101)
┌─────────────────────┐                    ┌─────────────────────┐
│ piper_inference     │                    │ piper_control       │
│ (Docker容器)        │                    │ (Docker容器)        │
│                     │                    │                     │
│ Contact-GraspNet    │    ZMQ PUB/SUB     │ IsaacSimForwarder   │
│ SAM + OpenCLIP      │ ←───────────────→  │ ROS2 Topics         │
│ inference_isaac_sim │    5555/5556       │ grasp_executor      │
└─────────────────────┘                    └─────────────────────┘
```

**Communication:**
- Port 5555: Jetson PUB → PGX SUB (sensor data: RGB/Depth)
- Port 5556: PGX PUB → Jetson SUB (grasp results + execution commands)

## Prerequisites

**DimOS must be running** with the agentic blueprint:
```bash
/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos run unitree-go2-agentic --daemon
```

**Infrastructure is auto-managed by scripts:**
- PGX inference container (`piper_inference`)
- Jetson CAN interface + Docker + control scripts

No manual startup required - `pick` skill handles it automatically.

## Commands

### Pick Up Object

**Instructions:** "抓取[物体]" / "拿起[物体]" / "拾取[物体]" / "pick up [object]"

**Command:**
```bash
dimos mcp call pick --json-args '{"object_name": "[object_en]"}'
```

**Examples:**
```bash
dimos mcp call pick --json-args '{"object_name": "bottle"}'
dimos mcp call pick --json-args '{"object_name": "cup"}'
dimos mcp call pick --json-args '{"object_name": "box"}'
```

**Common Objects:**
| English | 中文 |
|---------|------|
| bottle  | 瓶子 |
| cup     | 杯子 |
| box     | 盒子 |
| can     | 罐子 |
| apple   | 苹果 |

## Usage Examples

### User: 抓取瓶子
**Parsed:**
- Action: Pick up object
- Object: 瓶子 → "bottle"
- Command:
```bash
dimos mcp call pick --json-args '{"object_name": "bottle"}'
```

### User: 拿起杯子
**Parsed:**
- Object: 杯子 → "cup"
- Command:
```bash
dimos mcp call pick --json-args '{"object_name": "cup"}'
```

## Notes

1. **Auto startup**: `pick` automatically ensures PGX container and Jetson control are running
2. **Inference time**: SAM segmentation + Contact-GraspNet takes 10-30 seconds, but full execution can take up to 120s
3. **CLI timeout vs actual execution**: The `dimos mcp call` CLI has 30s timeout. If timeout occurs, the task may still be running in background. **Check logs to verify actual status:**
   ```bash
   LOG_DIR=~/.local/state/dimos/logs
   LATEST=$(ls -td "$LOG_DIR"/*/ | head -1)
   tail -10 "${LATEST}main.jsonl" | jq -r '.event'
   ```
4. **If pick fails**:
   - Check object is visible to camera
   - Try alternative object names (e.g., "water bottle" vs "bottle")
   - Ensure object is within arm workspace
5. **Verify skills loaded**: `dimos mcp list-tools` should show `pick`
6. **If tool not found**: Ensure DimOS is running with `unitree-go2-agentic` blueprint
7. **Object name interpretation**: Use exactly what the user specifies. If user says "抓取瓶子", use `"bottle"` not `"绿色瓶子"` from previous context. Never assume object names from earlier commands.

## Scripts Reference

Located in `dimos/robot/manipulators/piper/scripts/`:

| Script | Purpose |
|--------|---------|
| `piper_start_inference.sh` | Ensure PGX `piper_inference` container running |
| `piper_run_inference.sh [object] [ip]` | Run single-step Contact-GraspNet inference |
| `piper_start_jetson.sh [ip]` | Ensure Jetson CAN + Docker + control ready |

Can be run directly for manual control:
```bash
cd /home/emergeos/Share_pgx/ZLP/dimos/dimos/robot/manipulators/piper/scripts
./piper_start_inference.sh        # 启动PGX感知容器
./piper_start_jetson.sh           # 启动Jetson控制
./piper_run_inference.sh bottle   # 执行单步抓取
```