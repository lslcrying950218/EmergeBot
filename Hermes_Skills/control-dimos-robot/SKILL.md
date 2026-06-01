---
name: control-dimos-robot
description: Complete guide for controlling DimOS robot through natural language commands
version: 1.0.0
author: Hermes
tags: [dimos, robot-control, natural-language]
---

# DimOS Robot Control Commands

## Quick Reference

| User Command | Action | MCP Command |
|--------------|--------|-------------|
| "控制机械狗自主探索" | Start autonomous exploration | `dimos mcp call begin_exploration` |
| "寻找 [物体] 并移动到它面前" | Detect + navigate to static object | `look_out_for` + `auto_explore` + `navigate_to_object` |
| "寻找 [人] 并跟踪" | Detect + follow person | `look_out_for` + `auto_explore` + `follow_person` |
| "停止跟踪" | Stop following | `dimos mcp call stop_following` |
| "停止探索" | Stop exploration | `dimos mcp call end_exploration` |

## Static Object Navigation

**Format:**
```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["中文描述", "English description"],
  "auto_explore": true,
  "then": {
    "name": "navigate_to_object",
    "arguments": {
      "query": "English description",
      "goal_distance": 1.0
    }
  }
}'
```

**Common Objects:**
- 书架 → bookshelf
- 墙壁 → wall
- 椅子 → chair
- 桌子 → table
- 瓶子 → bottle
- 箱子 → box
- 鞋子 → shoes

**Example:** "寻找绿色瓶子并移动到它面前"
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["绿色瓶子", "green bottle"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "green bottle", "goal_distance": 1.0}}}'
```

## Person Tracking

**Format:**
```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["中文描述"],
  "then": {
    "name": "follow_person",
    "arguments": {
      "query": "中文描述"
    }
  },
  "auto_explore": true
}'
```

**Common Person Descriptions:**
- 穿灰色衣服的人
- 长头发男子
- 穿黑色鞋子的人
- 穿凉鞋的人
- 拿绿色杯子的人
- 穿绿色衣服的人
- 穿灰色裤子的人

## Important Notes

1. **NEVER** call `begin_exploration` then `navigate_to_object` separately — this stops exploration without detection
2. Always use `look_out_for` with `auto_explore: true` for detection + navigation
3. `goal_distance` controls final stopping distance from object (default 0.5m, recommended 1.0m)
4. For static objects: use `navigate_to_object`
5. For dynamic targets (people): use `follow_person`

## Common Workflow

**Autonomous Exploration Loop:**
1. Start: `dimos mcp call begin_exploration`
2. Detect & Track: `look_out_for` with appropriate target
3. Lost: `stop_following` → continue exploring
4. Stop all: `dimos mcp call end_exploration`