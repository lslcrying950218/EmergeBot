"use client";

import React, { useEffect, useRef } from "react";

interface VideoStreamProps {
  hostname: string;
  port?: number;
  className?: string;
}

/**
 * 完全独立的视频流组件 - 不依赖任何外部状态，不会因父组件重渲染而断流
 */
function VideoStream({ hostname, port = 7782, className = "" }: VideoStreamProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const hasConnected = useRef(false);

  useEffect(() => {
    if (!containerRef.current || hasConnected.current) return;

    // 只在第一次挂载时创建图片元素
    hasConnected.current = true;

    const img = document.createElement('img');
    img.src = `http://${hostname}:${port}/video_feed/video`;
    img.alt = "Robot Camera Feed";
    img.className = className;
    img.style.cssText = 'width: 100%; height: 100%; object-fit: cover; background: black;';

    containerRef.current.appendChild(img);
    imgRef.current = img;

    return () => {
      // 清理时移除图片
      if (imgRef.current && imgRef.current.parentNode) {
        imgRef.current.parentNode.removeChild(imgRef.current);
      }
      hasConnected.current = false;
    };
  }, [hostname, port, className]);

  return (
    <div ref={containerRef} className="w-full h-full" style={{ background: 'black' }} />
  );
}

export default React.memo(VideoStream);
