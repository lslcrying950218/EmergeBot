"use client";

import React, { useEffect, useState } from "react";
import { Panel, Group, Separator } from "react-resizable-panels";
import Header from "@/components/layout/Header";
import PerceptionPanel from "@/components/panels/PerceptionPanel";
import TaskPanel from "@/components/panels/TaskPanel";
import StatusPanel from "@/components/panels/StatusPanel";
import { useDimosStore } from "@/store/useDimosStore";

export const ResizeHandle = ({ orientation = "horizontal" }: { orientation?: "horizontal" | "vertical" }) => (
  <>
    <style jsx global>{`
      [data-panel-resize-handle-enabled] {
        outline: none !important;
      }
      [data-panel-resize-handle-focus] {
        outline: none !important;
      }
    `}</style>
    <Separator
      className={`${orientation === "horizontal" ? "w-1.5" : "h-1.5"} flex items-center justify-center group transition-all duration-300 outline-none`}
    >
      <div className={`
        ${orientation === "horizontal" ? "w-0.5 h-12 group-hover:h-full" : "h-0.5 w-12 group-hover:w-full"}
        bg-zinc-800 rounded-full group-hover:bg-blue-500 group-hover:shadow-[0_0_8px_rgba(59,130,246,0.6)] group-data-[dragging=true]:bg-blue-600 transition-all duration-300
      `} />
    </Separator>
  </>
);

export default function Home() {
  const { connect, disconnect, bridgeConnected, dimosLive, hermesConnected } = useDimosStore();
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setMounted(true);
    connect();
    return () => disconnect();
  }, [connect, disconnect]);

  return (
    <div className="flex flex-col h-full w-full overflow-hidden bg-[#0a0c10]">
      <Header />

      <main className="flex-1 overflow-hidden p-4">
        <Group orientation="horizontal">
          {/* Left Column */}
          <Panel defaultSize={30} minSize={20}>
            <div className="h-full pr-1">
              <PerceptionPanel />
            </div>
          </Panel>

          <ResizeHandle orientation="horizontal" />

          {/* Middle Column */}
          <Panel defaultSize={45} minSize={30}>
            <div className="h-full px-1">
              <TaskPanel />
            </div>
          </Panel>

          <ResizeHandle orientation="horizontal" />

          {/* Right Column */}
          <Panel defaultSize={25} minSize={15}>
            <div className="h-full pl-1">
              <StatusPanel />
            </div>
          </Panel>
        </Group>
      </main>

      {/* Status indicators at bottom */}
      {mounted && !bridgeConnected && (
        <div className="absolute bottom-4 left-4 px-2 py-1 bg-red-900/80 text-white text-[10px] rounded border border-red-500 animate-pulse z-[100]">
          OFFLINE: DIMOS BRIDGE NOT RUNNING
        </div>
      )}
      {mounted && bridgeConnected && !dimosLive && (
        <div className="absolute bottom-4 left-4 px-2 py-1 bg-amber-900/80 text-white text-[10px] rounded border border-amber-500 z-[100]">
          STANDBY: BRIDGE CONNECTED, NO DIMOS DATA
        </div>
      )}
      {mounted && !hermesConnected && (
        <div className="absolute bottom-4 left-72 px-2 py-1 bg-amber-900/80 text-white text-[10px] rounded border border-amber-500 z-[100]">
          OFFLINE: HERMES AGENT NOT CONNECTED
        </div>
      )}
    </div>
  );
}