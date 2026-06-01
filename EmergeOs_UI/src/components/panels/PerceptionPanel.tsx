"use client";

import React, { useState, useEffect, useRef } from "react";
import dynamic from "next/dynamic";
import { Panel, Group } from "react-resizable-panels";
import { ResizeHandle } from "@/app/page";
import { useDimosStore } from "@/store/useDimosStore";
import VideoStream from "@/components/VideoStream";
import {
  Video,
  Map as MapIcon,
  Camera,
  Navigation,
  Maximize2,
  Eye,
  Play,
  Pause,
  Square,
  AlertOctagon
} from "lucide-react";

const VoxelMap3D = dynamic(() => import("./VoxelMap3D"), { ssr: false });

const PerceptionPanel: React.FC = () => {
  const { costmap, robotPose, bridgeConnected, dimosLive, videoLive, telemetry, startDimos, stopDimos, interruptHermes, emergencyStop, hermesConnected, hermesBusy, sendChatMessage } = useDimosStore();
  const [mapMode, setMapMode] = useState<'2D' | '3D'>('2D');
  const [mounted, setMounted] = useState(false);
  const [hostname, setHostname] = useState('localhost');
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const videoContainerRef = useRef<HTMLDivElement>(null);

  // 只在挂载时获取 hostname
  useEffect(() => {
    if (typeof window !== 'undefined') {
      setHostname(window.location.hostname);
    }
  }, []);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setMounted(true);
  }, []);

  const toggleFullscreen = () => {
    if (videoContainerRef.current) {
      if (!document.fullscreenElement) {
        videoContainerRef.current.requestFullscreen().catch(err => {
          console.error(`全屏切换失败: ${err.message}`);
        });
      } else {
        document.exitFullscreen();
      }
    }
  };

  // 2D canvas rendering effect with cover mode
  useEffect(() => {
    if (!costmap || mapMode !== '2D' || !canvasRef.current || !mapContainerRef.current) return;

    const grid = costmap.grid;
    const shape = costmap.shape;
    if (!grid || !shape) return;

    const [mapHeight, mapWidth] = shape;
    const canvas = canvasRef.current;
    const container = mapContainerRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Get container dimensions with device pixel ratio for sharp rendering
    const dpr = window.devicePixelRatio || 1;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;

    // Set canvas actual pixel size
    canvas.width = containerWidth * dpr;
    canvas.height = containerHeight * dpr;

    // Scale context for HiDPI
    ctx.scale(dpr, dpr);

    // Create offscreen canvas for costmap at original resolution
    const offscreen = document.createElement('canvas');
    offscreen.width = mapWidth;
    offscreen.height = mapHeight;
    const offCtx = offscreen.getContext('2d');
    if (!offCtx) return;

    // Draw costmap to offscreen canvas
    const imgData = offCtx.createImageData(mapWidth, mapHeight);
    for (let i = 0; i < grid.length; i++) {
      const val = grid[i];
      const idx = i * 4;
      if (val === 255) {
        imgData.data[idx] = 30; imgData.data[idx+1] = 30; imgData.data[idx+2] = 40;
      } else if (val > 50) {
        imgData.data[idx] = 200; imgData.data[idx+1] = 50; imgData.data[idx+2] = 50;
      } else {
        imgData.data[idx] = 100; imgData.data[idx+1] = 100; imgData.data[idx+2] = 120;
      }
      imgData.data[idx+3] = 255;
    }
    offCtx.putImageData(imgData, 0, 0);

    // Calculate cover scaling
    const scaleX = containerWidth / mapWidth;
    const scaleY = containerHeight / mapHeight;
    const scale = Math.max(scaleX, scaleY); // cover mode

    const drawWidth = mapWidth * scale;
    const drawHeight = mapHeight * scale;
    const offsetX = (containerWidth - drawWidth) / 2;
    const offsetY = (containerHeight - drawHeight) / 2;

    // Clear and draw scaled costmap
    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, containerWidth, containerHeight);
    ctx.drawImage(offscreen, offsetX, offsetY, drawWidth, drawHeight);

    // Draw robot position with same transform
    if (robotPose) {
      const res = costmap.resolution || 0.1;
      const originX = costmap.origin?.c?.[0] || 0;
      const originY = costmap.origin?.c?.[1] || 0;
      const mapX = (robotPose.x - originX) / res;
      const mapY = mapHeight - (robotPose.y - originY) / res;

      // Apply cover transform
      const screenX = offsetX + mapX * scale;
      const screenY = offsetY + mapY * scale;

      ctx.beginPath();
      ctx.arc(screenX, screenY, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#3b82f6';
      ctx.fill();
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [costmap, robotPose, mapMode]);

  return (
    <Group orientation="vertical" className="h-full">
      {/* Real-time Perception */}
      <Panel defaultSize={35} minSize={20}>
        <section className="h-full bg-card border border-border rounded-lg overflow-hidden flex flex-col group/panel">
          <div className="px-4 py-2 border-b border-border flex justify-between items-center bg-zinc-900/50">
            <h2 className="text-[10px] font-bold uppercase tracking-widest text-zinc-500 flex items-center gap-2">
              <Video size={12} className="text-blue-500" /> 实时感知
            </h2>
            <div className="flex gap-1 bg-black/40 p-0.5 rounded-md border border-zinc-800">
              <button className="text-[9px] px-2.5 py-1 bg-zinc-800 text-white rounded shadow-sm font-medium transition-all">主视角</button>
              <button className="text-[9px] px-2.5 py-1 text-zinc-500 hover:text-zinc-300 rounded font-medium transition-all">机械臂相机</button>
            </div>
          </div>
          <div ref={videoContainerRef} className="flex-1 bg-black flex items-center justify-center overflow-hidden relative">
            {/* 视频流 - 使用完全独立的组件，不受 zustand 状态影响 */}
            {mounted && bridgeConnected ? (
              <VideoStream hostname={hostname} port={7782} className="w-full h-full object-cover" />
            ) : null}

            <div className="absolute inset-0 flex flex-col items-center justify-center pointer-events-none">
              {!bridgeConnected && (
                <div className="text-zinc-800 text-xs font-mono uppercase tracking-[0.3em] select-none flex flex-col items-center gap-2">
                  <div className="w-12 h-12 border-2 border-zinc-900 rounded-full flex items-center justify-center border-t-zinc-800 animate-spin">
                    <Camera size={20} className="text-zinc-900" />
                  </div>
                  Stream Initializing
                </div>
              )}
            </div>

            <div className={`absolute top-3 left-3 z-10 px-2 py-1 bg-black/60 backdrop-blur-md rounded border flex items-center gap-2 ${mounted && videoLive ? 'border-green-500/30' : mounted && bridgeConnected ? 'border-amber-500/30' : 'border-red-500/30'}`}>
              <div className={`w-1.5 h-1.5 rounded-full ${mounted && videoLive ? 'bg-green-500 animate-pulse' : mounted && bridgeConnected ? 'bg-amber-500' : 'bg-red-500'}`}></div>
              <span className={`text-[9px] font-mono font-bold uppercase tracking-tighter ${mounted && videoLive ? 'text-green-500' : mounted && bridgeConnected ? 'text-amber-500' : 'text-red-500'}`}>
                {mounted && videoLive ? `FPS: ${telemetry?.fps || 0} | CAM-1 ● 实时` : mounted && bridgeConnected ? '待机: 无视频流' : 'NO SIGNAL | DISCONNECTED'}
              </span>
            </div>

            <button
              onClick={toggleFullscreen}
              className="absolute top-3 right-3 z-10 p-1.5 bg-black/40 hover:bg-black/80 rounded border border-white/5 text-zinc-400 opacity-0 group-hover/panel:opacity-100 transition-opacity"
            >
              <Maximize2 size={12} />
            </button>

            <div className="absolute bottom-0 left-0 right-0 p-4 bg-gradient-to-t from-black/90 via-black/40 to-transparent flex justify-between items-end text-[10px] text-zinc-400 font-mono pointer-events-none">
              <div className="flex flex-col gap-1">
                <span className="flex items-center gap-1.5"><Eye size={10} className="text-zinc-600" /> 分辨率: {mounted && videoLive ? (telemetry?.res || '---') : 'N/A'}</span>
              </div>
            </div>
          </div>
        </section>
      </Panel>

      <ResizeHandle orientation="vertical" />

      {/* Semantic Mapping */}
      <Panel defaultSize={45} minSize={20}>
        <section className="h-full bg-card border border-border rounded-lg overflow-hidden flex flex-col">
          <div className="px-4 py-2 border-b border-border flex justify-between items-center bg-zinc-900/50">
            <h2 className="text-[10px] font-bold uppercase tracking-widest text-zinc-500 flex items-center gap-2">
              <MapIcon size={12} className="text-blue-500" /> 语义地图
            </h2>
            <div className="flex gap-1 bg-black/40 p-0.5 rounded-md border border-zinc-800">
              <button
                onClick={() => setMapMode('3D')}
                className={`text-[9px] px-2.5 py-1 rounded shadow-sm font-medium transition-all ${mapMode === '3D' ? 'bg-zinc-800 text-white' : 'text-zinc-500 hover:text-zinc-300'}`}
              >3D</button>
              <button
                onClick={() => setMapMode('2D')}
                className={`text-[9px] px-2.5 py-1 rounded shadow-sm font-medium transition-all ${mapMode === '2D' ? 'bg-zinc-800 text-white' : 'text-zinc-500 hover:text-zinc-300'}`}
              >2D</button>
            </div>
          </div>
          <div ref={mapContainerRef} className="flex-1 bg-black relative flex items-center justify-center overflow-hidden">
            {mapMode === '2D' ? (
              <canvas
                ref={canvasRef}
                className="absolute inset-0 w-full h-full"
              />
            ) : (
              <VoxelMap3D robotPose={robotPose} />
            )}

            {!costmap && !dimosLive && bridgeConnected && (
              <div className="text-zinc-600 text-[10px] font-mono uppercase tracking-[0.2em]">
                等待地图数据...
              </div>
            )}

            {videoLive && !dimosLive && bridgeConnected && (
              <div className="absolute top-3 left-3 z-10 px-2 py-1 bg-amber-900/60 backdrop-blur-md rounded border border-amber-500/30">
                <span className="text-[9px] font-mono font-bold uppercase tracking-tighter text-amber-400">
                  视频正常 · 等待遥测
                </span>
              </div>
            )}

            <div className="absolute bottom-3 right-3 z-10 flex flex-col gap-2">
              <button className="p-2 bg-black/60 border border-white/5 rounded text-zinc-400 hover:text-white transition-colors">
                <Navigation size={14} />
              </button>
              <button className="p-2 bg-black/60 border border-white/5 rounded text-zinc-400 hover:text-white transition-colors">
                <Maximize2 size={14} />
              </button>
            </div>
          </div>
        </section>
      </Panel>

      <ResizeHandle orientation="vertical" />

      {/* Task Control Quick Actions */}
      <Panel defaultSize={20} minSize={15}>
        <section className="h-full bg-card border border-border rounded-lg p-4 flex flex-col">
          <h2 className="text-[10px] font-bold uppercase tracking-widest text-zinc-500 mb-3 flex items-center gap-2">
            <Navigation size={12} className="text-blue-500" /> 任务控制 (Controls)
          </h2>
          <div className="grid grid-cols-2 gap-2 flex-1">
            <button
              onClick={() => startDimos('unitree-go2-agentic', false)}
              disabled={!bridgeConnected}
              className="p-2.5 bg-black/40 border border-white/5 rounded text-zinc-500 hover:bg-green-900/30 hover:text-green-500 hover:border-green-500/30 transition-all text-[10px] flex flex-col items-center justify-center gap-1.5 group disabled:opacity-50 disabled:cursor-not-allowed">
              <Play size={14} className="group-hover:scale-110 transition-transform" />
              启动系统
            </button>
            <button
              onClick={() => interruptHermes()}
              disabled={!hermesConnected || !hermesBusy}
              className="p-2.5 bg-black/40 border border-white/5 rounded text-zinc-500 hover:bg-amber-900/30 hover:text-amber-500 hover:border-amber-500/30 transition-all text-[10px] flex flex-col items-center justify-center gap-1.5 group disabled:opacity-50 disabled:cursor-not-allowed">
              <Pause size={14} className="group-hover:scale-110 transition-transform" />
              暂停会话
            </button>
            <button
              onClick={() => {
                interruptHermes();
                // 延迟发送消息，等待中断完成
                setTimeout(() => {
                  sendChatMessage('中止任务');
                }, 500);
              }}
              disabled={!hermesConnected || !hermesBusy}
              className="p-2.5 bg-black/40 border border-white/5 rounded text-zinc-500 hover:bg-red-900/30 hover:text-red-500 hover:border-red-500/30 transition-all text-[10px] flex flex-col items-center justify-center gap-1.5 group disabled:opacity-50 disabled:cursor-not-allowed">
              <Square size={14} className="group-hover:scale-110 transition-transform" />
              中止任务
            </button>
            <button
              onClick={() => emergencyStop()}
              className="p-2.5 bg-black/40 border border-white/5 rounded text-zinc-500 hover:bg-orange-900/30 hover:text-orange-500 hover:border-orange-500/30 transition-all text-[10px] flex flex-col items-center justify-center gap-1.5 group">
              <AlertOctagon size={14} className="group-hover:scale-110 transition-transform" />
              紧急停止
            </button>
          </div>
        </section>
      </Panel>
    </Group>
  );
};

export default PerceptionPanel;