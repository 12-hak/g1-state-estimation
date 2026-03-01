import React, { useState, useEffect, useCallback, useRef } from 'react';
import { MapCanvas } from './components/MapCanvas';
import { StatusPanel } from './components/StatusPanel';
import { ControlPanel } from './components/ControlPanel';
import { useWebSocket } from './hooks/useWebSocket';
import type { RobotPose, MapData, SlamStatus, Waypoint, WSMessage } from './types';

const WS_PORT = (import.meta as any).env?.VITE_WS_PORT || '9090';
const WS_URL = `ws://${window.location.hostname}:${WS_PORT}`;

export const App: React.FC = () => {
  const { connected, send, onMessage } = useWebSocket(WS_URL);
  const [pose, setPose] = useState<RobotPose | null>(null);
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [status, setStatus] = useState<SlamStatus | null>(null);
  const [mapPoints, setMapPoints] = useState<number[][]>([]);
  const [scanPoints, setScanPoints] = useState<number[][]>([]);
  const [pointSize, setPointSize] = useState(2);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);

  /** After startup grace, only draw green scan when ICP converged + enough points (avoids green spin when lost). */
  const MIN_ALIGN_POINTS = 50;
  const alignmentOk = !!(status?.localization_valid && mapPoints.length >= MIN_ALIGN_POINTS);
  const [startupGrace, setStartupGrace] = useState(true);
  const [mode, setMode] = useState<'navigate' | 'waypoint'>('navigate');
  const lastPoseAtRef = useRef<number | null>(null);
  const lastYawRef = useRef<number | null>(null);

  // For first 10s show scan regardless of alignment so map can build; then apply alignment rule
  useEffect(() => {
    const t = setTimeout(() => setStartupGrace(false), 10000);
    return () => clearTimeout(t);
  }, []);

  useEffect(() => {
    return onMessage((data: WSMessage) => {
      if (data.type === 'pose') {
        setPose({ x: data.x, y: data.y, z: data.z, yaw: data.yaw });
        if (data.status) setStatus(data.status);
        lastPoseAtRef.current = performance.now();
        lastYawRef.current = data.yaw;
      } else if (data.type === 'status') {
        setScanPoints(data.scan_points);
        if (data.point_size) setPointSize(data.point_size);
        if (data.status) setStatus(data.status);
      } else if (data.type === 'map_cloud') {
        setMapPoints(data.map_points);
        if (data.status) setStatus(data.status);
      } else if (data.type === 'map') {
        setMapData({
          width: data.width,
          height: data.height,
          resolution: data.resolution,
          origin_x: data.origin_x,
          origin_y: data.origin_y,
          data: data.data,
          path: data.path,
        });
      }
    });
  }, [onMessage]);

  const handleClickMap = useCallback((x: number, y: number) => {
    send({ type: 'navigate', x, y, yaw: 0 });
  }, [send]);

  const handleAddWaypoint = useCallback((x: number, y: number) => {
    setWaypoints(prev => [...prev, {
      id: `wp_${Date.now()}`,
      x, y, yaw: 0,
    }]);
  }, []);

  const handleRemoveWaypoint = useCallback((id: string) => {
    setWaypoints(prev => prev.filter(w => w.id !== id));
  }, []);

  const handleExecuteWaypoints = useCallback(() => {
    send({
      type: 'waypoints',
      points: waypoints.map(w => ({ x: w.x, y: w.y, yaw: w.yaw })),
      loop: false,
    });
  }, [send, waypoints]);

  const handleCancel = useCallback(() => {
    send({ type: 'cancel' });
  }, [send]);

  const handleSaveMap = useCallback(() => {
    send({ type: 'save_map', name: '' });
  }, [send]);

  return (
    <div style={{
      width: '100vw', height: '100vh', overflow: 'hidden',
      position: 'relative', background: '#1a1a2e',
    }}>
      <MapCanvas
        pose={pose}
        mapData={mapData}
        mapPoints={mapPoints}
        scanPoints={scanPoints}
        alignmentOk={alignmentOk}
        startupGrace={startupGrace}
        pointSize={pointSize}
        waypoints={waypoints}
        onClickMap={handleClickMap}
        onAddWaypoint={handleAddWaypoint}
        mode={mode}
      />
      <StatusPanel pose={pose} status={status} connected={connected} />
      <ControlPanel
        mode={mode}
        onModeChange={setMode}
        waypoints={waypoints}
        onClearWaypoints={() => setWaypoints([])}
        onExecuteWaypoints={handleExecuteWaypoints}
        onRemoveWaypoint={handleRemoveWaypoint}
        onCancel={handleCancel}
        onSaveMap={handleSaveMap}
      />
    </div>
  );
};
