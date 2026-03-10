/**
 * Web view matching RViz point_lio.rviz: map cloud, scan (no decay, refresh as we move), path, robot.
 * Navigation UI deferred.
 */

import React, { useState, useEffect, useCallback, useRef, useMemo } from 'react';
import { MapCanvas, type ScanFrame } from './components/MapCanvas';
import { FullMap3D } from './components/FullMap3D';
import { useWebSocket } from './hooks/useWebSocket';
import type { RobotPose, WSMessage } from './types';

const WS_PORT = (import.meta as any).env?.VITE_WS_PORT ?? '9090';
const WS_URL = `ws://${window.location.hostname}:${WS_PORT}`;
/** Max scan frames to keep in memory (trail is persistent, no time decay). */
const SCAN_FRAMES_MAX = 120;
const RIGHT_WINDOW_POINTS = 100000;
/** Voxel size (m) for accumulating map layer so it never decays. */
const MAP_VOXEL_M = 0.025;
const SAVE_MAP_NAME = 'save_map';

// Persist accumulated clouds outside React so they survive remounts (Strict Mode, HMR, etc.)
const persistedMap = new Map<string, number[]>();
const persistedTrail = new Map<string, number[]>();

export const App: React.FC = () => {
  const { connected, onMessage, send } = useWebSocket(WS_URL);
  const [pose, setPose] = useState<RobotPose | null>(null);
  const [mapPoints, setMapPoints] = useState<number[][]>([]);
  const [scanFrames, setScanFrames] = useState<ScanFrame[]>([]);
  const [rollingPoints, setRollingPoints] = useState<number[][]>([]);
  const [trajectory, setTrajectory] = useState<number[][]>([]);
  const [saveCount, setSaveCount] = useState(0);
  const [saveBusy, setSaveBusy] = useState(false);
  const trailRef = useRef(persistedTrail);
  const mapRef = useRef(persistedMap);
  const [trailVersion, setTrailVersion] = useState(0);
  const onTrailUpdate = useCallback(() => setTrailVersion(v => v + 1), []);

  const pointsFor3D = useMemo(() => {
    if (rollingPoints.length > 0) return rollingPoints;
    const trailPoints = Array.from(trailRef.current.values());
    if (trailPoints.length > 0) return trailPoints.slice(-RIGHT_WINDOW_POINTS);
    if (mapPoints.length > 0) return mapPoints.slice(-RIGHT_WINDOW_POINTS);
    return [];
  }, [rollingPoints, trailVersion, mapPoints]);

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
        const incoming = (data.scan_points ?? []).filter((p: number[]) => p.length >= 2);
        setScanFrames(prev => {
          const next = [...prev, { points: data.scan_points, receivedAt: Date.now() }];
          return next.slice(-SCAN_FRAMES_MAX);
        });
        if (incoming.length > 0) {
          setRollingPoints(prev => {
            const next = [...prev, ...incoming];
            return next.length > RIGHT_WINDOW_POINTS ? next.slice(-RIGHT_WINDOW_POINTS) : next;
          });
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
  }, [onMessage]);

  const onSaveMap = useCallback(() => {
    if (saveBusy) return;
    setSaveBusy(true);
    send({
      type: 'save_map',
      name: SAVE_MAP_NAME,
    });
    setSaveCount(v => v + 1);
    setTimeout(() => setSaveBusy(false), 2500);
  }, [saveBusy, send]);

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
          onClick={onSaveMap}
          style={{
            background: saveBusy ? '#555' : '#0f3460',
            color: '#fff',
            border: 'none',
            borderRadius: 4,
            padding: '6px 10px',
            fontSize: 12,
            cursor: 'pointer',
          }}
        >
          Save Map
        </button>
        <span style={{ color: '#aaa', fontSize: 12, fontFamily: 'sans-serif' }}>
          right window: {pointsFor3D.length.toLocaleString()} / {RIGHT_WINDOW_POINTS.toLocaleString()} · saves: {saveCount} · file: {SAVE_MAP_NAME}
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
