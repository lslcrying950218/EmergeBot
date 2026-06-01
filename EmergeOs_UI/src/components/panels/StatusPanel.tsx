"use client";

import React from "react";
import { Panel, Group } from "react-resizable-panels";
import { ResizeHandle } from "@/app/page";
import { useDimosStore } from "@/store/useDimosStore";
import {
  Battery,
  Activity,
  Clock,
  Weight,
  ShieldCheck,
  AlertTriangle
} from "lucide-react";

// ============ 配置项 ============
const BATTERY_WARNING_THRESHOLD = 40; // 电量告警阈值 (%)
// ================================

const StatusPanel: React.FC = () => {
  const { telemetry, robotPose, bridgeConnected, dimosLive, videoLive } = useDimosStore();

  // 电量告警状态
  const isBatteryWarning = telemetry?.battery != null
    && telemetry.battery < BATTERY_WARNING_THRESHOLD
    && !telemetry.batteryStale;

  return (
    <Group orientation="vertical" className="h-full">
      {/* Robot Status */}
      <Panel defaultSize={35} minSize={15}>
        <section className="h-full bg-card border border-border rounded-lg overflow-hidden flex flex-col">
          <div className="px-4 py-2 border-b border-border bg-zinc-900/50 flex justify-between items-center">
            <h2 className="text-xs font-semibold uppercase tracking-wider text-zinc-400">机器人状态</h2>
            <span className={`text-[10px] uppercase ${dimosLive ? 'text-green-500' : videoLive ? 'text-blue-500' : bridgeConnected ? 'text-amber-500' : 'text-zinc-500'}`}>
              {dimosLive ? 'Unitree G2' : videoLive ? 'VIDEO ONLY' : bridgeConnected ? 'STANDBY' : 'OFFLINE'}
            </span>
          </div>
          <div className="p-4 flex flex-col gap-4 flex-1 overflow-y-auto custom-scrollbar">
            <div className="grid grid-cols-2 gap-3">
              <div className="bg-black/20 p-2 rounded border border-zinc-800/50">
                <div className="flex items-center gap-2 text-[10px] text-zinc-500 uppercase mb-1">
                  <Battery size={10} className={
                    telemetry?.batteryStale ? "text-zinc-600" :
                    isBatteryWarning ? "text-red-500 animate-pulse" :
                    telemetry?.battery != null && telemetry.battery >= BATTERY_WARNING_THRESHOLD ? "text-green-500" :
                    "text-amber-500"
                  } /> 电量
                </div>
                <div className="flex items-center gap-2">
                  <div className="flex-1 h-1.5 bg-zinc-800 rounded-full overflow-hidden">
                    <div
                      className={`h-full transition-all ${
                        telemetry?.batteryStale ? 'bg-zinc-600' :
                        isBatteryWarning ? 'bg-red-500' :
                        telemetry?.battery != null && telemetry.battery >= BATTERY_WARNING_THRESHOLD ? 'bg-green-500' :
                        'bg-amber-500'
                      }`}
                      style={{ width: telemetry?.battery != null && !telemetry.batteryStale ? `${telemetry.battery}%` : '0%' }}
                    ></div>
                  </div>
                  <span className={`text-xs font-mono ${isBatteryWarning ? 'text-red-400' : 'text-zinc-300'}`}>
                    {telemetry?.battery != null && !telemetry.batteryStale ? `${telemetry.battery}%` : '--'}
                  </span>
                </div>
              </div>

              <div className="bg-black/20 p-2 rounded border border-zinc-800/50">
                <div className="flex items-center gap-2 text-[10px] text-zinc-500 uppercase mb-1">
                  <Activity size={10} className={dimosLive ? "text-blue-500" : "text-zinc-700"} /> 速度
                </div>
                <div className="text-xs font-mono text-zinc-300">{dimosLive ? `${telemetry?.speed || 0.00} m/s` : '--'}</div>
              </div>
              <div className="bg-black/20 p-2 rounded border border-zinc-800/50">
                <div className="flex items-center gap-2 text-[10px] text-zinc-500 uppercase mb-1">
                  <Clock size={10} className="text-zinc-500" /> 运行时长
                </div>
                <div className="text-xs font-mono text-zinc-300">{dimosLive ? '02:36:41' : '--'}</div>
              </div>
            </div>
            <div className="pt-2 border-t border-zinc-800/50 grid grid-cols-3 text-center gap-2">
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">Roll</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive ? `${telemetry?.roll || 0.0}°` : '--'}</div>
              </div>
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">Pitch</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive ? `${telemetry?.pitch || 0.0}°` : '--'}</div>
              </div>
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">Yaw</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive ? `${telemetry?.yaw || 0.0}°` : '--'}</div>
              </div>
            </div>
            <div className="grid grid-cols-3 text-center gap-2">
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">X</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive && robotPose ? `${robotPose.x.toFixed(2)}` : '--'}</div>
              </div>
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">Y</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive && robotPose ? `${robotPose.y.toFixed(2)}` : '--'}</div>
              </div>
              <div>
                <div className="text-[9px] text-zinc-600 uppercase">Z</div>
                <div className="text-[11px] font-mono text-zinc-400">{dimosLive && robotPose ? `${robotPose.z.toFixed(2)}` : '--'}</div>
              </div>
            </div>
          </div>
        </section>
      </Panel>

      <ResizeHandle orientation="vertical" />

      {/* Robotic Arm Status */}
      <Panel defaultSize={40} minSize={15}>
        <section className="h-full bg-card border border-border rounded-lg overflow-hidden flex flex-col">
          <div className="px-4 py-2 border-b border-border bg-zinc-900/50 flex justify-between items-center">
            <h2 className="text-xs font-semibold uppercase tracking-wider text-zinc-400">机械臂状态</h2>
            <span className={`text-[9px] uppercase font-medium ${dimosLive ? 'text-green-500' : bridgeConnected ? 'text-amber-500' : 'text-zinc-700'}`}>
              {dimosLive ? '已就绪' : bridgeConnected ? '待机' : '未连接'}
            </span>
          </div>
          <div className="p-4 space-y-4 flex-1 overflow-y-auto custom-scrollbar">
            <div className="grid grid-cols-2 gap-y-2 gap-x-4 text-[11px]">
              {['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].map(id => (
                <div key={id} className="flex justify-between border-b border-zinc-800/30 pb-1">
                  <span className="text-zinc-500 font-mono">{id}</span>
                  <span className="text-zinc-300 font-mono">{dimosLive ? '0.0°' : '--'}</span>
                </div>
              ))}
            </div>
            <div className="grid grid-cols-2 gap-3 pt-2">
              <div className="bg-black/20 p-2 rounded border border-zinc-800/50">
                <div className="flex items-center gap-2 text-[9px] text-zinc-500 uppercase mb-1">
                  <Weight size={9} /> 负载
                </div>
                <div className="text-xs font-mono text-zinc-300">{dimosLive ? '0.0 kg' : '--'}</div>
              </div>
              <div className="bg-black/20 p-2 rounded border border-zinc-800/50">
                <div className="flex items-center gap-2 text-[9px] text-zinc-500 uppercase mb-1">
                  <ShieldCheck size={9} className={dimosLive ? "text-green-500" : "text-zinc-700"} /> 可用状态
                </div>
                <div className="text-xs font-medium text-zinc-500">{dimosLive ? '可用' : '不可用'}</div>
              </div>
            </div>
          </div>
        </section>
      </Panel>

      <ResizeHandle orientation="vertical" />

      {/* Alerts & Messages */}
      <Panel defaultSize={25} minSize={10}>
        <section className={`h-full rounded-lg overflow-hidden flex flex-col ${
          isBatteryWarning
            ? 'bg-red-900/10 border border-red-900/30'
            : 'bg-card border border-border'
        }`}>
          <div className={`px-4 py-2 border-b flex justify-between items-center ${
            isBatteryWarning
              ? 'bg-red-900/20 border-red-900/30'
              : 'bg-zinc-900/50 border-border'
          }`}>
            <h2 className={`text-xs font-semibold uppercase tracking-wider ${
              isBatteryWarning ? 'text-red-400' : 'text-zinc-400'
            }`}>告警与消息</h2>
            {isBatteryWarning ? (
              <span className="text-[10px] text-red-500 animate-pulse font-bold tracking-tighter flex items-center gap-1">
                <AlertTriangle size={10} /> WARNING
              </span>
            ) : (
              <span className="text-[10px] text-zinc-500 uppercase font-medium">正常</span>
            )}
          </div>
          <div className="p-4 flex-1 overflow-y-auto custom-scrollbar">
            {isBatteryWarning ? (
              <div className="flex gap-3">
                <div className="shrink-0 w-8 h-8 rounded bg-red-900/30 flex items-center justify-center text-red-500">
                  <AlertTriangle size={16} />
                </div>
                <div className="flex flex-col gap-1">
                  <div className="text-[10px] text-zinc-500 font-mono">
                    {new Date().toLocaleTimeString('zh-CN', { hour12: false })}
                  </div>
                  <p className="text-[11px] text-red-300/80 leading-relaxed">
                    电量不足警告：当前电量 {telemetry?.battery}%，请尽快充电以避免系统关机。
                  </p>
                </div>
              </div>
            ) : (
              <div className="h-full flex items-center justify-center">
                <p className="text-[10px] text-zinc-500 uppercase tracking-widest">无告警信息</p>
              </div>
            )}
          </div>
        </section>
      </Panel>
    </Group>
  );
};

export default StatusPanel;