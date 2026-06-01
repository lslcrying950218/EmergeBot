"use client";

import React, { useState, useEffect } from "react";
import { useDashboardStore } from "@/store/useDashboardStore";
import { useDimosStore } from "@/store/useDimosStore";
import { Settings, User, Zap, Globe, Clock } from "lucide-react";

const Header: React.FC = () => {
  const { latency } = useDashboardStore();
  const { bridgeConnected, dimosLive } = useDimosStore();
  const [systemTime, setSystemTime] = useState<string>("");

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();
      const year = now.getFullYear();
      const month = String(now.getMonth() + 1).padStart(2, '0');
      const day = String(now.getDate()).padStart(2, '0');
      const hours = String(now.getHours()).padStart(2, '0');
      const minutes = String(now.getMinutes()).padStart(2, '0');
      const seconds = String(now.getSeconds()).padStart(2, '0');
      setSystemTime(`${year}-${month}-${day} ${hours}:${minutes}:${seconds}`);
    };

    updateTime();
    const interval = setInterval(updateTime, 1000);
    return () => clearInterval(interval);
  }, []);

  return (
    <header className="flex items-center justify-between h-14 shrink-0 px-6 bg-card/50 backdrop-blur-md border-b border-border z-50">
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 bg-blue-600 rounded-lg flex items-center justify-center shadow-lg shadow-blue-900/40">
            <Zap size={18} className="text-white fill-current" />
          </div>
          <div className="flex flex-col">
            <h1 className="text-lg font-bold tracking-tight leading-none">
              EMERGE <span className="text-blue-500">OS</span>
            </h1>
            <span className="text-[9px] font-medium text-zinc-500 uppercase tracking-[0.2em] mt-1">
              Robot Brain Operating System
            </span>
          </div>
        </div>
      </div>

      <div className="flex items-center gap-10 text-xs">
        <div className="flex items-center gap-6">
          <div className="flex flex-col items-start gap-0.5">
            <span className="text-[9px] text-zinc-500 uppercase font-bold tracking-wider flex items-center gap-1">
              <Globe size={10} /> Connectivity
            </span>
            <div className="flex items-center gap-2">
              <div className={`w-1.5 h-1.5 rounded-full ${dimosLive ? 'bg-green-500 animate-pulse' : bridgeConnected ? 'bg-amber-500' : 'bg-red-500'}`}></div>
              <span className={dimosLive ? 'text-zinc-200' : bridgeConnected ? 'text-amber-400' : 'text-red-400'}>
                {dimosLive ? '在线 (Online)' : bridgeConnected ? '待机 (Standby)' : '离线 (Offline)'}
              </span>
            </div>
          </div>

          <div className="flex flex-col items-start gap-0.5">
            <span className="text-[9px] text-zinc-500 uppercase font-bold tracking-wider">Latency</span>
            <span className="font-mono text-blue-400">{latency}ms</span>
          </div>

          <div className="flex flex-col items-start gap-0.5">
            <span className="text-[9px] text-zinc-500 uppercase font-bold tracking-wider flex items-center gap-1">
              <Clock size={10} /> System Time
            </span>
            <span className="font-mono text-zinc-300">{systemTime || "---"}</span>
          </div>
        </div>

        <div className="flex items-center gap-3 border-l border-zinc-800 pl-8">
          <div className="flex flex-col items-end">
            <span className="text-[9px] text-zinc-500 uppercase font-bold">Operator</span>
            <span className="text-zinc-200 font-medium tracking-tight">admin_root</span>
          </div>
          <div className="flex gap-2">
            <button className="w-9 h-9 rounded-xl bg-zinc-800/50 hover:bg-zinc-700 hover:text-white transition-all flex items-center justify-center text-zinc-400 border border-zinc-700/30">
              <User size={16} />
            </button>
            <button className="w-9 h-9 rounded-xl bg-zinc-800/50 hover:bg-zinc-700 hover:text-white transition-all flex items-center justify-center text-zinc-400 border border-zinc-700/30">
              <Settings size={16} />
            </button>
          </div>
        </div>
      </div>
    </header>
  );
};

export default Header;