# EmergeBot

物理 Agent OS 的现实应用 —— 基于大语言模型的具身智能机器人系统。

## 项目简介

EmergeBot 是一套完整的具身智能机器人解决方案，涵盖从底层硬件设计、机器人操作系统、自然语言交互到高级抓取规划的完整技术栈。系统支持通过自然语言指令控制机器人执行探索、导航、目标检测与抓取等复杂任务。

## 系统架构

```
EmergeBot/
├── dimos/                      # 核心机器人操作系统 (Agentive OS)
├── EmergeOs_UI/                # 实时监控与控制仪表盘
├── Hermes_Skills/              # 自然语言技能集 (MCP 协议)
├── Real_Pier_Graspnet_Pytorch/ # 6-DoF 抓取生成系统
└── 3D_Printing_Models/         # 硬件结构件 3D 打印模型
```

## 模块说明

### dimos — 机器人操作系统

DimOS 是面向物理空间的 Agent 操作系统，提供通用的机器人控制能力。

- **模块系统**：自治子系统通过类型化流（typed streams）通信
- **Blueprints**：组合式机器人栈构建系统
- **Agent 系统**：基于 LLM 的智能体，可通过技能控制机器人（支持 GPT-4o / Ollama 本地模型）
- **导航**：SLAM、路径规划、障碍物规避
- **感知**：目标检测、追踪、空间记忆
- **多语言支持**：Python（主要）、C++、Lua、TypeScript
- **通信协议**：LCM、ROS2、DDS、共享内存
- **支持平台**：Unitree Go2 / G1 / B1 四足机器人、XArm 机械臂、MAVLink 无人机

### EmergeOs_UI — 实时控制仪表盘

基于 Next.js 的实时机器人监控与控制界面。

- 实时视频流与语义地图可视化
- 任务执行监控与自然语言指令输入
- 硬件遥测数据显示
- **技术栈**：Next.js 16 + React 19 + TypeScript + Tailwind CSS v4 + Three.js + Zustand + Socket.io

### Hermes_Skills — 自然语言技能集

将自然语言指令转化为 DimOS MCP 命令的技能集合，支持中英文。

| 技能 | 版本 | 功能 |
|------|------|------|
| explore-find-navigate | v4.0.0 | 自主探索、目标检测、追踪跟随、复合任务链 |
| piper-arm-control | v2.1.0 | 机械臂抓取控制，集成 Contact-GraspNet |
| dimos-natural-command | v1.1.0 | 自然语言到 DimOS MCP 命令的转换 |
| control-dimos-robot | - | 基础机器人控制指令 |

### Real_Pier_Graspnet_Pytorch — 6-DoF 抓取生成

基于 Contact-GraspNet 的 PyTorch 实现，用于杂乱场景中的抓取姿态预测。

- 从 3D 点云实时生成 6-DoF 抓取姿态
- 集成 SAM + OpenCLIP 进行分割与目标识别
- 支持 Isaac Sim 仿真环境
- 训练基于 ACRONYM 数据集
- **硬件需求**：推理需 >=8GB 显存，训练需 >=24GB 显存

### 3D_Printing_Models — 硬件结构件

机器人平台的 3D 打印结构件模型，包括：

- Unitree Go2 四足机器人改装件（背板、支架）
- Mid360 激光雷达安装件
- RealSense 相机末端执行器安装件
- 机械臂底板及支撑结构

## 快速开始

各模块的详细安装与使用说明请参阅对应目录下的 README：

- [dimos 使用文档](dimos/README.md)
- [EmergeOs_UI](EmergeOs_UI/README.md)
- [Hermes_Skills](Hermes_Skills/)
- [Real_Pier_Graspnet_Pytorch](Real_Pier_Graspnet_Pytorch/README.md)

## 许可证

本项目基于 [MIT License](LICENSE) 开源。

Copyright (c) 2026 lslcrying950218
