---
name: dimos-natural-command
description: Convert natural language commands to DimOS MCP commands
version: 1.1.0
author: Hermes
tags: [dimos, natural-language, mcp]
triggers:
  - "控制机械狗"
  - "自主探索"
  - "寻找"
  - "找.*导航"
  - "找.*移动"
  - "找.*跟踪"
  - "找.*抓取"
  - "寻找.*抓"
  - "搜索.*抓"
  - "并抓取"
  - "并拿起"
  - "抓起来"
  - "拿起"
  - "pick"
  - "grab"
  - "find and pick"
---

# Natural Language Command for DimOS

Convert natural language instructions to correct DimOS MCP commands.

## ⚠️ 重要：抓取意图识别

**当用户指令包含"抓取"、"拿起"、"抓"、"捡"等词时，必须生成完整的三层链！**

| 用户意图 | 错误输出 | 正确输出 |
|---------|---------|---------|
| 寻找X并抓取 | `look_out_for → navigate_to_object` (缺少pick) | `look_out_for → navigate_to_object → pick` |
| 找X并拿起 | 只有导航，不抓取 | 导航后自动触发抓取 |

**错误示例（用户说抓取但只生成导航）：**
```bash
# ❌ 错误：缺少 pick
dimos mcp call look_out_for --json-args '{"description_of_things": ["绿色瓶子"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "绿色瓶子"}}}'
```

**正确示例：**
```bash
# ✅ 正确：包含完整三层链
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

## Command Types

### 1. Autonomous Exploration
**Instructions:** "控制机械狗自主探索" / "开始探索" / "开始自主探索"
**Command:** `dimos mcp call begin_exploration`

### 2. Navigate to Static Object
**Instructions:** "寻找 [物体] 并移动到它面前" / "导航到 [物体]" / "找到 [物体]"
**Command:** 
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["[object_desc]", "[object_en]"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "[object_en]", "goal_distance": [distance]}}}'
```

**IMPORTANT:** Must use `look_out_for` with `auto_explore: true` and `then: navigate_to_object`. Do NOT call `begin_exploration` + `navigate_to_object` separately — that will stop exploration without starting detection.

**Common Objects:**
- bookshelf/shelf → 书架
- wall → 墙壁
- chair → 椅子
- table → 桌子
- bottle → 瓶子
- box → 箱子
- shoe → 鞋子

**Parameters:**
- `query`: Object description in English
- `goal_distance`: Distance to stop from object (meters), default 0.5, recommended 1.0

### 3. Follow Person
**Instructions:** "寻找 [人员描述] 并跟踪" / "跟踪 [人员描述]"
**Command:** 
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["[person_desc]"], "then": {"name": "follow_person", "arguments": {"query": "[person_desc]"}}, "auto_explore": true}'
```

**Common Person Descriptions:**
- 穿灰色衣服的人 (person in gray clothes)
- 长头发男子 (man with long hair)
- 穿黑色鞋子的人 (person wearing black shoes)
- 穿凉鞋的人 (person wearing sandals)
- 拿绿色杯子的人 (person holding green cup)
- 穿绿色衣服的人 (person in green clothes)
- 穿灰色裤子的人 (person wearing gray pants)

### 4. Explore + Detect + Navigate + Pick (Full Chain)
**Instructions:** "寻找 [物体] 并抓取" / "搜索 [物体] 拿起它" / "找 [物体] 并抓起来"
**Command:** 
```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["[object_desc]", "[object_en]"],
  "auto_explore": true,
  "then": {
    "name": "navigate_to_object",
    "arguments": {"query": "[object_en]", "goal_distance": 0.5},
    "then": {
      "name": "pick",
      "arguments": {}
    }
  }
}'
```

**Parameters:**
- `goal_distance`: Recommended 0.5m for pick (closer than navigate-only)
- `pick.arguments`: Empty `{}` — system auto-fills `object_name` from detection label

**Common Objects for Pick:**
- 瓶子 → bottle
- 绿色瓶子 → green bottle
- 水瓶 → water bottle
- 杯子 → cup
- 红色可乐 → red cola can

### 5. Direct Pick (Robot Already Positioned)
**Instructions:** "抓取 [物体]" / "拿起 [物体]" / "抓 [物体]" (when robot is already in front of object)
**Command:** 
```bash
dimos mcp call pick --json-args '{"object_name": "[object_desc]"}'
```

**When to use:** 
- Robot has already navigated to the object
- User explicitly says "抓取" or "拿起" without "寻找" or "导航"
- **Do NOT use full chain** when robot is already positioned

**Example:**
- User: "抓取瓶子" (robot already in front of bottle)
- Command: `dimos mcp call pick --json-args '{"object_name": "绿色瓶子"}'`

**Note:** If pick fails with "No valid grasps found", the object may be:
- Out of camera field of view
- Not in arm's reachable workspace
- Not detectable by ContactGraspNet

### 6. Stop Following
**Instructions:** "停止跟踪" / "停止"
**Command:** `dimos mcp call stop_following`

### 7. Stop Exploration
**Instructions:** "停止探索"
**Command:** `dimos mcp call end_exploration`

## Usage Examples

### User: 寻找书架并移动到它面前
**Parsed:**
- Action: Explore + detect + navigate to static object
- Object: 书架 → "bookshelf"
- Command:
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["书架", "bookshelf"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "bookshelf", "goal_distance": 1.0}}}'
```

### User: 寻找绿色瓶子并移动到它面前
**Parsed:**
- Action: Explore + detect + navigate to static object
- Object: 绿色瓶子 → "green bottle"
- Command:
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["绿色瓶子", "green bottle"], "auto_explore": true, "then": {"name": "navigate_to_object", "arguments": {"query": "green bottle", "goal_distance": 1.0}}}'
```

### User: 控制机械狗自主探索
**Parsed:**
- Action: Start autonomous exploration
- Command: `dimos mcp call begin_exploration`

### User: 寻找穿灰色衣服的人并跟踪
**Parsed:**
- Action: Detect and follow person
- Target: 穿灰色衣服的人
- Command:
```bash
dimos mcp call look_out_for --json-args '{"description_of_things": ["穿灰色衣服的人"], "then": {"name": "follow_person", "arguments": {"query": "穿灰色衣服的人"}}, "auto_explore": true}'
```

### User: 寻找绿色瓶子并抓取
**Parsed:**
- Action: Explore + detect + navigate + pick (full chain)
- Object: 绿色瓶子 → "green bottle"
- Command:
```bash
dimos mcp call look_out_for --json-args '{
  "description_of_things": ["绿色瓶子", "green bottle"],
  "auto_explore": true,
  "then": {
    "name": "navigate_to_object",
    "arguments": {"query": "green bottle", "goal_distance": 0.5},
    "then": {
      "name": "pick",
      "arguments": {}
    }
  }
}'
```

## Notes
1. **Static objects**: Use `look_out_for` + `auto_explore: true` + `then: navigate_to_object`
2. **Dynamic targets (people)**: Use `look_out_for` + `auto_explore: true` + `then: follow_person`
3. `goal_distance` affects final stopping distance, default 0.5m, recommended 1.0m for better direction estimation
4. **NEVER call `begin_exploration` then `navigate_to_object` separately** — this stops exploration without detection running
5. **Direct Pick vs Full Chain**: 
   - "寻找X并抓取" → Full chain (explore + navigate + pick)
   - "抓取X" / "拿起X" → Direct pick (robot already positioned)
   - Ask user if unclear which to use
