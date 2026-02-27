import React from 'react';
import type { RobotPose, SlamStatus, DebugTelemetry } from '../types';

interface Props {
  pose: RobotPose | null;
  status: SlamStatus | null;
  connected: boolean;
  debug: DebugTelemetry | null;
}

export const StatusPanel: React.FC<Props> = ({ pose, status, connected, debug }) => {
  return (
    <div style={{
      position: 'absolute', top: 12, left: 12, zIndex: 1000,
      background: 'rgba(10, 10, 30, 0.85)',
      color: '#e0e0e0',
      padding: '12px 16px',
      borderRadius: 8,
      fontFamily: 'monospace',
      fontSize: 13,
      minWidth: 220,
      boxShadow: '0 2px 12px rgba(0,0,0,0.5)',
      pointerEvents: 'none',
    }}>
      <div style={{ fontSize: 16, fontWeight: 'bold', marginBottom: 8, color: '#00ff41' }}>
        G1 SLAM Navigator
      </div>
      <div style={{ color: connected ? '#00ff41' : '#ff4444', marginBottom: 6 }}>
        {connected ? 'CONNECTED' : 'DISCONNECTED'}
      </div>
      {pose && (
        <div style={{ marginBottom: 6 }}>
          <div>X: {pose.x.toFixed(3)}m</div>
          <div>Y: {pose.y.toFixed(3)}m</div>
          <div>Yaw: {(pose.yaw * 180 / Math.PI).toFixed(1)} deg</div>
        </div>
      )}
      {status && status.quality !== undefined && (
        <div style={{ borderTop: '1px solid #333', paddingTop: 6, marginTop: 4 }}>
          <div>Map: {status.mapping_active ? 'ACTIVE' : 'IDLE'}</div>
          <div>Loc: {status.localization_valid ?
            <span style={{ color: '#00ff41' }}>VALID</span> :
            <span style={{ color: '#ff4444' }}>LOST</span>}
          </div>
          <div>Points: {(status.map_points ?? 0).toLocaleString()}</div>
          <div>Loops: {status.loop_closures ?? 0}</div>
          <div>Quality: {((status.quality ?? 0) * 100).toFixed(0)}%</div>
        </div>
      )}
      {debug && (
        <div style={{ borderTop: '1px solid #333', paddingTop: 6, marginTop: 6, color: '#9fd0ff' }}>
          <div style={{ color: '#c8e8ff', marginBottom: 2 }}>Debug Telemetry</div>
          <div>PoseHz: {debug.pose_hz.toFixed(1)} | ScanHz: {debug.scan_hz.toFixed(1)}</div>
          <div>CloudHz: {debug.map_cloud_hz.toFixed(1)} | Clients: {debug.ws_clients}</div>
          <div>Seq P/S/C: {debug.pose_seq}/{debug.scan_seq}/{debug.map_cloud_seq}</div>
          <div>Age ms P/S/C: {debug.pose_age_ms}/{debug.scan_age_ms}/{debug.map_cloud_age_ms}</div>
          <div>Yaw deg: {(debug.pose_yaw_deg ?? 0).toFixed(1)} | dYaw/s: {(debug.pose_yaw_rate_deg_s ?? 0).toFixed(1)}</div>
          <div>Pts Scan/Map: {debug.scan_points}/{debug.map_points}</div>
          <div>Voxel: {debug.map_voxel_size.toFixed(2)}m</div>
        </div>
      )}
    </div>
  );
};
