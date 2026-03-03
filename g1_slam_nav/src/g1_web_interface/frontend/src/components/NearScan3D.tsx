/**
 * Small 3D overlay: near (<2 m) scan as point cloud with orbit controls.
 * Robot at origin; height-based vertex colours (rainbow like 2D).
 */

import React, { useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import type { RobotPose, Point } from '../types';
import { pointZ } from '../types';

const NEAR_RADIUS_M = 2.0;
const NEAR_VOXEL_M = 0.02;

export interface ScanFrame {
  points: Point[];
  receivedAt: number;
}

interface NearScan3DProps {
  scanFrames: ScanFrame[];
  pose: RobotPose | null;
}

function heightToRgb(z: number, zMin: number, zMax: number): [number, number, number] {
  if (zMax <= zMin) return [0.6, 0.6, 0.6];
  const t = Math.max(0, Math.min(1, (z - zMin) / (zMax - zMin)));
  const hue = (1 - t) * 240; // 240 = blue at low, 0 = red at high
  const c = 0.55 * 0.85;
  const x = c * (1 - Math.abs(((hue / 60) % 2) - 1));
  const m = 0.55 - c;
  let r = 0, g = 0, b = 0;
  if (hue < 60) { r = c; g = x; b = 0; }
  else if (hue < 120) { r = x; g = c; b = 0; }
  else if (hue < 180) { r = 0; g = c; b = x; }
  else if (hue < 240) { r = 0; g = x; b = c; }
  else if (hue < 300) { r = x; g = 0; b = c; }
  else { r = c; g = 0; b = x; }
  return [r + m, g + m, b + m];
}

function PointCloud({ points, pose, centerX, centerY }: { points: Point[]; pose: RobotPose | null; centerX: number; centerY: number }) {
  const rx = centerX;
  const ry = centerY;

  const { positions, colors } = useMemo(() => {
    const positions: number[] = [];
    const colors: number[] = [];
    if (points.length === 0) {
      return { positions: new Float32Array(0), colors: new Float32Array(0) };
    }
    let zMin = Infinity, zMax = -Infinity;
    for (const p of points) {
      const z = pointZ(p);
      if (!Number.isNaN(z)) { zMin = Math.min(zMin, z); zMax = Math.max(zMax, z); }
    }
    if (zMax <= zMin) zMax = zMin + 1;

    for (const p of points) {
      const px = Number(p[0] ?? 0);
      const py = Number(p[1] ?? 0);
      const pz = Number(p[2] ?? 0);
      positions.push(px - rx, py - ry, pz);
      const [r, g, b] = heightToRgb(pz, zMin, zMax);
      colors.push(r, g, b);
    }
    return {
      positions: new Float32Array(positions),
      colors: new Float32Array(colors),
    };
  }, [points, centerX, centerY]);

  if (positions.length === 0) return null;

  const pointCount = positions.length / 3;
  return (
    <points key={`pointcloud-${pointCount}`}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={pointCount}
          array={positions}
          itemSize={3}
        />
        <bufferAttribute
          attach="attributes-color"
          count={pointCount}
          array={colors}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial size={0.06} vertexColors depthWrite={true} sizeAttenuation />
    </points>
  );
}

function RobotMarker() {
  return (
    <group>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.08, 16, 16]} />
        <meshBasicMaterial color="#00ff41" />
      </mesh>
      <mesh position={[0.14, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.06, 0.18, 8]} />
        <meshBasicMaterial color="#00cc33" />
      </mesh>
    </group>
  );
}

export const NearScan3D: React.FC<NearScan3DProps> = ({ scanFrames, pose }) => {
  const nearPoints = useMemo(() => {
    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r2 = NEAR_RADIUS_M * NEAR_RADIUS_M;
    const hasPose = pose != null;
    const seen = new Set<string>();
    const out: Point[] = [];
    for (const frame of scanFrames.slice(-2)) {
      for (const p of frame.points) {
        if (p.length < 2) continue;
        const px = Number(p[0]), py = Number(p[1]);
        if (hasPose && (px - rx) * (px - rx) + (py - ry) * (py - ry) > r2) continue;
        const vn = NEAR_VOXEL_M;
        const k = `${Math.floor(px / vn)},${Math.floor(py / vn)},${Math.floor(Number(p[2] ?? 0) / vn)}`;
        if (seen.has(k)) continue;
        seen.add(k);
        out.push(p);
      }
    }
    return out;
  }, [scanFrames, pose?.x, pose?.y]);

  const centerX = pose?.x ?? (nearPoints.length > 0
    ? nearPoints.reduce((s, p) => s + Number(p[0] ?? 0), 0) / nearPoints.length
    : 0);
  const centerY = pose?.y ?? (nearPoints.length > 0
    ? nearPoints.reduce((s, p) => s + Number(p[1] ?? 0), 0) / nearPoints.length
    : 0);

  return (
    <div style={{
      position: 'absolute',
      bottom: 16,
      right: 16,
      width: 300,
      height: 250,
      borderRadius: 8,
      overflow: 'hidden',
      background: 'rgba(0,0,0,0.7)',
      border: '1px solid rgba(255,255,255,0.2)',
      zIndex: 10,
    }}>
      <Canvas
        camera={{ position: [2.5, 2.5, 2.5], fov: 50 }}
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
      >
        <color attach="background" args={['#0a0a0a']} />
        <ambientLight intensity={0.6} />
        <pointLight position={[5, 5, 5]} intensity={1} />
        <PointCloud points={nearPoints} pose={pose} centerX={centerX} centerY={centerY} />
        <RobotMarker />
        <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
      </Canvas>
    </div>
  );
};
