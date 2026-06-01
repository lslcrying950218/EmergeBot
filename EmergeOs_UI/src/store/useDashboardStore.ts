import { create } from 'zustand';

interface Telemetry {
  battery: number;
  temperature: number;
  speed: number;
  runtime: string;
  load: number;
  orientation: { roll: number; pitch: number; yaw: number };
  joints: { [key: string]: string };
}

interface LogEntry {
  id: string;
  timestamp: string;
  message: string;
  status: 'success' | 'warning' | 'error' | 'processing';
}

interface DashboardState {
  telemetry: Telemetry;
  taskProgress: number;
  logs: LogEntry[];
  isOnline: boolean;
  latency: number;
  
  // Actions
  updateTelemetry: (data: Partial<Telemetry>) => void;
  addLog: (message: string, status?: LogEntry['status']) => void;
  setTaskProgress: (progress: number) => void;
  setOnlineStatus: (status: boolean) => void;
}

export const useDashboardStore = create<DashboardState>((set) => ({
  isOnline: true,
  latency: 18,
  taskProgress: 64,
  telemetry: {
    battery: 70,
    temperature: 41.6,
    speed: 0.42,
    runtime: '02:36:41',
    load: 4.8,
    orientation: { roll: -1.2, pitch: 1.8, yaw: 32.7 },
    joints: {
      J1: '-8.2°', J2: '-22.5°', J3: '-45.6°', 
      J4: '-68.3°', J5: '12.1°', J6: '3.4°'
    }
  },
  logs: [
    { id: '1', timestamp: '14:40:03', message: '机器人开始自主探索, 数据采集与特征提取。', status: 'success' },
    { id: '2', timestamp: '14:41:02', message: '检测到目标区域, 进行路径规划与执行。', status: 'success' },
    { id: '3', timestamp: '14:41:22', message: '目标确认成功, 锁定目标 ID: Person_0427。', status: 'success' },
  ],

  updateTelemetry: (data) => set((state) => ({
    telemetry: { ...state.telemetry, ...data }
  })),
  
  addLog: (message, status = 'processing') => set((state) => ({
    logs: [
      ...state.logs,
      {
        id: Math.random().toString(36).substr(2, 9),
        timestamp: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
        message,
        status
      }
    ].slice(-50) // Keep last 50 logs
  })),

  setTaskProgress: (progress) => set({ taskProgress: progress }),
  setOnlineStatus: (status) => set({ isOnline: status }),
}));
