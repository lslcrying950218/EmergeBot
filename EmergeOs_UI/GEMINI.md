# EMERGE OS Dashboard - Project Overview

## Project Purpose
EMERGE OS is a high-performance, real-time robot brain operating system dashboard. It provides operators with comprehensive situational awareness, including live perception feeds, semantic mapping, task control, and detailed hardware status monitoring.

## Tech Stack
- **Framework**: [Next.js](https://nextjs.org/) (App Router)
- **Library**: [React](https://reactjs.org/) with TypeScript
- **Styling**: [Tailwind CSS](https://tailwindcss.com/)
- **Icons**: [Lucide React](https://lucide.dev/) (Recommended)
- **State Management**: [Zustand](https://github.com/pmndrs/zustand) (Recommended for real-time updates)
- **Visualization**: [Three.js](https://threejs.org/) / [React Three Fiber](https://docs.pmnd.rs/react-three-fiber) (Planned for 3D mapping)

## Architecture & Layout
The application uses a fixed-height, full-screen three-column layout:
1. **Header**: Global system status, connectivity, and operator info.
2. **Left Panel**: Real-time perception (video) and semantic mapping visualization.
3. **Middle Panel**: Task execution monitoring, natural language command interface, and execution logs.
4. **Right Panel**: Hardware telemetry (battery, motors, joints) and system health alerts.

## Development Conventions
- **Componentization**: Break down panels into smaller, reusable components in `src/components`.
- **Styling**: Use Tailwind utility classes. Maintain the dark, high-contrast "cyber" aesthetic (Background: `#0a0c10`, Card: `#11141a`).
- **State**: Keep real-time telemetry state optimized to prevent unnecessary re-renders of the entire dashboard.
- **Types**: Ensure strict TypeScript typing for all telemetry data and task states.

## Getting Started
### Commands
- `npm run dev`: Start the development server.
- `npm run build`: Build for production.
- `npm run lint`: Run ESLint checks.

## Project Structure
- `src/app/`: Next.js App Router pages and global styles.
- `src/components/`: Reusable UI components (Dashboard panels, widgets).
- `src/hooks/`: Custom hooks for telemetry and data fetching.
- `src/store/`: Zustand stores for global state management.
## 数据流的接入
Emerge UI 需要接入两个软件的数据流
不要修改这两个软件内部的代码，可能需要的数据桥接器都放在UI的工程目录中
1. /home/emergeos/.hermes 的对话和思维链内容，并且用户需要与hermes在UI界面中交互
2. /home/emergeos/Share_pgx/ZLP/dimos 中的dimos执行日志，日志获取可以参考'/home/emergeos/Share_pgx/ZLP/dimos/AGENTS.md' 
