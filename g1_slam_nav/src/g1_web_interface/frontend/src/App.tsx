import React, { useState, useEffect, useCallback } from 'react';
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
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [mode, setMode] = useState<'navigate' | 'waypoint'>('navigate');

  useEffect(() => {
    return onMessage((data: WSMessage) => {
      if (data.type === 'pose') {
        setPose({ x: data.x, y: data.y, z: data.z, yaw: data.yaw });
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
