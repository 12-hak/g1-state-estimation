/**
 * Web view matching RViz point_lio.rviz: map cloud, scan (no decay, refresh as we move), path, robot.
 * Navigation UI deferred.
 */

import React, { useState, useEffect, useCallback, useRef, useMemo } from 'react';
import { MapCanvas, type ScanFrame } from './components/MapCanvas';
import { NearScan3D } from './components/NearScan3D';
import { FullMap3D } from './components/FullMap3D';
import { useWebSocket } from './hooks/useWebSocket';
import type { RobotPose, WSMessage } from './types';

const WS_PORT = (import.meta as any).env?.VITE_WS_PORT ?? '9090';
const WS_URL = `ws://${window.location.hostname}:${WS_PORT}`;
/** Max scan frames to keep in memory (trail is persistent, no time decay). */
const SCAN_FRAMES_MAX = 120;
/** Voxel size (m) for accumulating map layer so it never decays. */
const MAP_VOXEL_M = 0.025;
const BUILD_TARGET_POINTS = 100000;
const BUILD_LIVE_SAVE_NAME = 'build_live';

// Persist accumulated clouds outside React so they survive remounts (Strict Mode, HMR, etc.)
const persistedMap = new Map<string, number[]>();
const persistedTrail = new Map<string, number[]>();

export const App: React.FC = () => {
  const { connected, onMessage, send } = useWebSocket(WS_URL);
  const [pose, setPose] = useState<RobotPose | null>(null);
  const [mapPoints, setMapPoints] = useState<number[][]>([]);
  const [scanFrames, setScanFrames] = useState<ScanFrame[]>([]);
  const [trajectory, setTrajectory] = useState<number[][]>([]);
  const [buildMode, setBuildMode] = useState(false);
  const [buildPoints, setBuildPoints] = useState<number[][]>([]);
  const [buildSaveCount, setBuildSaveCount] = useState(0);
  const trailRef = useRef(persistedTrail);
  const mapRef = useRef(persistedMap);
  const saveCooldownRef = useRef(false);
  const [trailVersion, setTrailVersion] = useState(0);
  const onTrailUpdate = useCallback(() => setTrailVersion(v => v + 1), []);

  const pointsFor3D = useMemo(() => {
    if (buildMode) return buildPoints;
    if (mapPoints.length > 0) return mapPoints;
    const trailPoints = Array.from(trailRef.current.values());
    if (trailPoints.length > 0) return trailPoints;
    const out: number[][] = [];
    const seen = new Set<string>();
    for (const frame of scanFrames.slice(-2)) {
      for (const p of frame.points) {
        if (p.length < 2) continue;
        const k = `${Number(p[0])},${Number(p[1])},${Number(p[2] ?? 0)}`;
        if (seen.has(k)) continue;
        seen.add(k);
        out.push(p);
      }
    }
    return out;
  }, [buildMode, buildPoints, mapPoints, scanFrames, trailVersion]);

  useEffect(() => {
    if (mapRef.current.size > 0) {
      setMapPoints(Array.from(mapRef.current.values()));
    }
  }, []);

  useEffect(() => {
    return onMessage((data: WSMessage) => {
      if (data.type === 'pose') {
        setPose({ x: data.x, y: data.y, z: data.z, yaw: data.yaw });
      } else if (data.type === 'status') {
        setScanFrames(prev => {
          const next = [...prev, { points: data.scan_points, receivedAt: Date.now() }];
          return next.slice(-SCAN_FRAMES_MAX);
        });
        if (buildMode) {
          const incoming = (data.scan_points ?? []).filter((p: number[]) => p.length >= 2);
          const reachedTarget = buildPoints.length + incoming.length >= BUILD_TARGET_POINTS;
          setBuildPoints(prev => {
            if (incoming.length === 0) return prev;
            const next = [...prev, ...incoming];
            return next.length > BUILD_TARGET_POINTS ? next.slice(-BUILD_TARGET_POINTS) : next;
          });
          if (reachedTarget && !saveCooldownRef.current) {
            saveCooldownRef.current = true;
            setBuildSaveCount(prev => prev + 1);
            send({
              type: 'save_map',
              name: BUILD_LIVE_SAVE_NAME,
            });
            setTimeout(() => {
              saveCooldownRef.current = false;
            }, 5000);
          }
        }
      } else if (data.type === 'map_cloud') {
        const map = mapRef.current;
        const v = MAP_VOXEL_M;
        const key = (p: number[]) => {
          const x = p[0] ?? 0, y = p[1] ?? 0, z = p[2] ?? 0;
          return `${Math.floor(x / v)} ${Math.floor(y / v)} ${Math.floor(z / v)}`;
        };
        let added = 0;
        for (const p of data.map_points ?? []) {
          if (p.length < 2) continue;
          const k = key(p);
          if (!map.has(k)) {
            map.set(k, [...p]);
            added++;
          }
        }
        if (added > 0) {
          setMapPoints(Array.from(map.values()));
        }
      } else if (data.type === 'map' && data.trajectory) {
        setTrajectory(data.trajectory);
      }
    });
  }, [buildMode, buildPoints.length, onMessage, send]);

  const toggleBuildMode = useCallback(() => {
    setBuildMode(prev => {
      const next = !prev;
      if (!next) {
        setBuildPoints([]);
      } else {
        setBuildSaveCount(0);
      }
      return next;
    });
  }, []);

  return (
    <div style={{
      width: '100vw',
      height: '100vh',
      overflow: 'hidden',
      display: 'flex',
      flexDirection: 'row',
      background: '#000',
    }}>
      <div style={{ flex: '1 1 50%', position: 'relative', minWidth: 0, height: '100%' }}>
        <MapCanvas
          pose={pose}
          mapPoints={mapPoints}
          scanFrames={scanFrames}
          trajectory={trajectory}
          trailRef={trailRef}
          trailVersion={trailVersion}
          onTrailUpdate={onTrailUpdate}
        />
        <NearScan3D scanFrames={scanFrames} pose={pose} />
      </div>
      <div style={{ flex: '1 1 50%', minWidth: 0, position: 'relative', height: '100%' }}>
        <FullMap3D points={pointsFor3D} pose={pose} />
      </div>
      <div style={{
        position: 'absolute',
        top: 36,
        left: 8,
        display: 'flex',
        alignItems: 'center',
        gap: 8,
        zIndex: 30,
      }}>
        <button
          onClick={toggleBuildMode}
          style={{
            background: buildMode ? '#00aa44' : '#333',
            color: '#fff',
            border: 'none',
            borderRadius: 4,
            padding: '6px 10px',
            fontSize: 12,
            cursor: 'pointer',
          }}
        >
          Build Mode: {buildMode ? 'ON' : 'OFF'}
        </button>
        <span style={{ color: '#aaa', fontSize: 12, fontFamily: 'sans-serif' }}>
          window: {buildPoints.length.toLocaleString()} / {BUILD_TARGET_POINTS.toLocaleString()} · saves: {buildSaveCount} · file: {BUILD_LIVE_SAVE_NAME}
        </span>
      </div>
      <div style={{
        position: 'absolute',
        top: 8,
        left: 8,
        display: 'flex',
        alignItems: 'center',
        gap: 8,
        color: '#aaa',
        fontSize: 12,
        fontFamily: 'sans-serif',
        zIndex: 20,
        pointerEvents: 'none',
      }}>
        <span style={{
          width: 8,
          height: 8,
          borderRadius: 4,
          background: connected ? '#0f0' : '#f00',
        }} />
        {connected ? 'Connected' : 'Disconnected'}
        <span style={{ opacity: 0.6 }}>· no decay</span>
      </div>
    </div>
  );
};
