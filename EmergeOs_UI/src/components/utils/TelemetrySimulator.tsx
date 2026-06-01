"use client";

import { useEffect } from "react";
import { useDashboardStore } from "@/store/useDashboardStore";

export const TelemetrySimulator = () => {
  const { updateTelemetry, addLog, setTaskProgress } = useDashboardStore();

  useEffect(() => {
    // Simulate telemetry updates
    const telemetryInterval = setInterval(() => {
      updateTelemetry({
        speed: +(Math.random() * 0.5 + 0.2).toFixed(2),
        orientation: {
          roll: +((Math.random() - 0.5) * 2).toFixed(1),
          pitch: +((Math.random() - 0.5) * 3).toFixed(1),
          yaw: +(30 + Math.random() * 5).toFixed(1),
        }
      });
    }, 2000);

    // Simulate occasional logs
    const logInterval = setInterval(() => {
      const messages = [
        "传感器数据校验通过。",
        "路径规划引擎已刷新。",
        "避障模块检测到静态物体。",
        "视觉识别锁定目标特征。",
      ];
      const msg = messages[Math.floor(Math.random() * messages.length)];
      addLog(msg, "success");
    }, 8000);

    // Simulate progress
    const progressInterval = setInterval(() => {
      setTaskProgress(Math.floor(Math.random() * 10 + 60));
    }, 5000);

    return () => {
      clearInterval(telemetryInterval);
      clearInterval(logInterval);
      clearInterval(progressInterval);
    };
  }, [updateTelemetry, addLog, setTaskProgress]);

  return null; // This is a logic-only component
};
