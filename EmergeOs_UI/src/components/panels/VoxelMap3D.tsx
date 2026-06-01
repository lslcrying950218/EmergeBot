"use client";

import React, { useMemo, useRef, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera, Center } from "@react-three/drei";
import * as THREE from "three";
import { useDimosStore } from "@/store/useDimosStore";

interface VoxelMap3DProps {
  robotPose: { x: number; y: number; z: number } | null;
}

const RobotMarker3D = ({ pose }: { pose: { x: number; y: number; z: number } | null }) => {
  if (!pose) return null;
  return (
    <group position={[pose.x, 0.4, -pose.y]}>
      <mesh><boxGeometry args={[0.4, 0.6, 0.4]} /><meshBasicMaterial color="#00ffff" wireframe /></mesh>
      <primitive object={new THREE.AxesHelper(1.0)} />
    </group>
  );
};

const GlobalMapVoxels = ({ points }: { points: [number, number, number][] }) => {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const dummy = useMemo(() => new THREE.Object3D(), []);
  const color = useMemo(() => new THREE.Color(), []);

  // Calculate height range for color mapping
  const { minZ, maxZ } = useMemo(() => {
    if (points.length === 0) return { minZ: 0, maxZ: 1 };
    let min = Infinity, max = -Infinity;
    points.forEach(p => {
      if (p[2] < min) min = p[2];
      if (p[2] > max) max = p[2];
    });
    return { minZ: min, maxZ: Math.max(min + 0.01, max) }; // Avoid div by zero
  }, [points]);

  useEffect(() => {
    if (!meshRef.current) return;

    points.forEach((p, i) => {
      dummy.position.set(p[0], p[2], -p[1]);
      dummy.scale.set(1, 1, 1);
      dummy.updateMatrix();
      meshRef.current!.setMatrixAt(i, dummy.matrix);

      // Color based on height: deep blue (low) -> cyan -> green -> yellow -> red (high)
      const t = Math.max(0, Math.min(1, (p[2] - minZ) / (maxZ - minZ)));
      // Interpolate from deep blue (#1e3a8a) through cyan to red
      if (t < 0.25) {
        // Deep blue to blue
        color.setRGB(0.12, 0.23 + t * 0.77, 0.54 + t * 0.46);
      } else if (t < 0.5) {
        // Blue to cyan
        const s = (t - 0.25) * 4;
        color.setRGB(0.12 + s * 0.26, 1, 1);
      } else if (t < 0.75) {
        // Cyan to yellow
        const s = (t - 0.5) * 4;
        color.setRGB(0.38 + s * 0.62, 1, 1 - s);
      } else {
        // Yellow to red
        const s = (t - 0.75) * 4;
        color.setRGB(1, 1 - s, 0);
      }
      meshRef.current!.setColorAt(i, color);
    });

    meshRef.current.instanceMatrix.needsUpdate = true;
    if (meshRef.current.instanceColor) meshRef.current.instanceColor.needsUpdate = true;
  }, [points, dummy, color, minZ, maxZ]);

  return (
    <instancedMesh ref={meshRef} args={[undefined, undefined, points.length]}>
      <boxGeometry args={[0.08, 0.08, 0.08]} />
      <meshBasicMaterial />
    </instancedMesh>
  );
};

const Empty3DState = () => (
  <mesh position={[0, 0.5, 0]}>
    <boxGeometry args={[2, 1, 2]} />
    <meshBasicMaterial color="#1a1a2e" wireframe />
  </mesh>
);

const VoxelMap3D: React.FC<VoxelMap3DProps> = ({ robotPose }) => {
  const { globalMapPoints } = useDimosStore();

  return (
    <div className="w-full h-full bg-black">
      <Canvas>
        <PerspectiveCamera makeDefault position={[10, 10, 10]} fov={45} />
        <OrbitControls makeDefault />
        <gridHelper args={[100, 50, "#333", "#111"]} position={[0, -0.05, 0]} />

        <Center top>
          {globalMapPoints && globalMapPoints.length > 0 ? (
            <GlobalMapVoxels points={globalMapPoints} />
          ) : (
            <Empty3DState />
          )}
          <RobotMarker3D pose={robotPose} />
        </Center>
      </Canvas>
    </div>
  );
};

export default VoxelMap3D;
