/**
 * Web view matching RViz point_lio.rviz: map cloud, scan (no decay, refresh as we move), path, robot.
 * Navigation UI deferred.
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { MapCanvas, type ScanFrame } from './components/MapCanvas';
import { useWebSocket } from './hooks/useWebSocket';
import type { RobotPose, WSMessage } from './types';

const WS_PORT = (import.meta as any).env?.VITE_WS_PORT ?? '9090';
const WS_URL = `ws://${window.location.hostname}:${WS_PORT}`;
/** Max scan frames to keep in memory (trail is persistent, no time decay). */
const SCAN_FRAMES_MAX = 120;
/** Voxel size (m) for accumulating map layer so it never decays. */
const MAP_VOXEL_M = 0.025;

// Persist accumulated clouds outside React so they survive remounts (Strict Mode, HMR, etc.)
const persistedMap = new Map<string, number[]>();
const persistedTrail = new Map<string, number[]>();

export const App: React.FC = () => {
  const { connected, onMessage } = useWebSocket(WS_URL);
  const [pose, setPose] = useState<RobotPose | null>(null);
  const [mapPoints, setMapPoints] = useState<number[][]>([]);
  const [scanFrames, setScanFrames] = useState<ScanFrame[]>([]);
  const [trajectory, setTrajectory] = useState<number[][]>([]);
  const trailRef = useRef(persistedTrail);
  const mapRef = useRef(persistedMap);
  const [trailVersion, setTrailVersion] = useState(0);
  const onTrailUpdate = useCallback(() => setTrailVersion(v => v + 1), []);

  // Rehydrate map state from persisted map after remount so map layer doesn't disappear
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
  }, [onMessage]);

  return (
    <div style={{
      width: '100vw',
      height: '100vh',
      overflow: 'hidden',
      position: 'relative',
      background: '#000',
    }}>
      <MapCanvas
        pose={pose}
        mapPoints={mapPoints}
        scanFrames={scanFrames}
        trajectory={trajectory}
        trailRef={trailRef}
        trailVersion={trailVersion}
        onTrailUpdate={onTrailUpdate}
      />
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
