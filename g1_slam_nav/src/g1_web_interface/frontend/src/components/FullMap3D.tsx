import React, { useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import type { RobotPose, Point } from '../types';
import { pointZ } from '../types';

const MAX_POINTS = 100000;
const VOXEL_M = 0.04;

function heightToRgb(z: number, zMin: number, zMax: number): [number, number, number] {
  if (zMax <= zMin) return [0.6, 0.6, 0.6];
  const t = Math.max(0, Math.min(1, (z - zMin) / (zMax - zMin)));
  const hue = (1 - t) * 240;
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

function FullMapPointCloud({ points, pose }: { points: Point[]; pose: RobotPose | null }) {
  const cx = pose?.x ?? 0;
  const cy = pose?.y ?? 0;
  const cz = pose?.z ?? 0;

  const { positions, colors, pointCount } = useMemo(() => {
    if (points.length === 0) {
      return { positions: new Float32Array(0), colors: new Float32Array(0), pointCount: 0 };
    }
    const pos: number[] = [];
    const col: number[] = [];
    let zMin = Infinity, zMax = -Infinity;
    const seen = new Set<string>();
    const voxel = VOXEL_M;
    const sampled: Point[] = [];
    for (const p of points) {
      const z = pointZ(p);
      if (!Number.isNaN(z)) { zMin = Math.min(zMin, z); zMax = Math.max(zMax, z); }
      const px = Number(p[0] ?? 0), py = Number(p[1] ?? 0), pz = Number(p[2] ?? 0);
      const k = `${Math.floor(px / voxel)},${Math.floor(py / voxel)},${Math.floor(pz / voxel)}`;
      if (seen.has(k)) continue;
      seen.add(k);
      sampled.push(p);
      if (sampled.length >= MAX_POINTS) break;
    }
    if (zMax <= zMin) zMax = zMin + 1;
    const n = sampled.length;
    for (let i = 0; i < n; i++) {
      const p = sampled[i];
      const px = Number(p[0] ?? 0), py = Number(p[1] ?? 0), pz = Number(p[2] ?? 0);
      pos.push(px - cx, py - cy, pz - cz);
      const [r, g, b] = heightToRgb(pz, zMin, zMax);
      // Age fade: newer points stay bright, older points dim.
      const fade = n > 1 ? (0.25 + 0.75 * (i / (n - 1))) : 1.0;
      col.push(r * fade, g * fade, b * fade);
    }
    return {
      positions: new Float32Array(pos),
      colors: new Float32Array(col),
      pointCount: sampled.length,
    };
  }, [points, cx, cy, cz]);

  if (pointCount === 0) return null;

  return (
    <points key={`cloud-${pointCount}`}>
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
      <pointsMaterial size={0.25} vertexColors depthWrite sizeAttenuation={false} />
    </points>
  );
}

function RobotMarker() {
  return (
    <group>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.15, 16, 16]} />
        <meshBasicMaterial color="#00ff41" />
      </mesh>
      <mesh position={[0.2, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.1, 0.25, 8]} />
        <meshBasicMaterial color="#00cc33" />
      </mesh>
    </group>
  );
}

interface FullMap3DProps {
  points: Point[];
  pose: RobotPose | null;
}

export const FullMap3D: React.FC<FullMap3DProps> = ({ points, pose }) => {
  const count = points.length;
  return (
    <div style={{ width: '100%', height: '100%', minHeight: 200, background: '#0a0a0a', position: 'relative' }}>
      <div style={{
        position: 'absolute',
        top: 8,
        left: 8,
        zIndex: 10,
        color: '#aaa',
        fontSize: 12,
        fontFamily: 'sans-serif',
      }}>
        Points: {count.toLocaleString()}
      </div>
      <Canvas
        camera={{ position: [8, 8, 8], fov: 50 }}
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
        style={{ display: 'block', width: '100%', height: '100%' }}
      >
        <color attach="background" args={['#0a0a0a']} />
        <ambientLight intensity={0.6} />
        <pointLight position={[20, 20, 20]} intensity={1} />
        <FullMapPointCloud points={points} pose={pose} />
        <RobotMarker />
        <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
      </Canvas>
    </div>
  );
};
