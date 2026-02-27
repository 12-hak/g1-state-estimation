import React from 'react';
import type { Waypoint } from '../types';

interface Props {
  mode: 'navigate' | 'waypoint';
  onModeChange: (mode: 'navigate' | 'waypoint') => void;
  waypoints: Waypoint[];
  onClearWaypoints: () => void;
  onExecuteWaypoints: () => void;
  onRemoveWaypoint: (id: string) => void;
  onCancel: () => void;
  onSaveMap: () => void;
}

const btnStyle: React.CSSProperties = {
  padding: '8px 14px',
  border: 'none',
  borderRadius: 4,
  cursor: 'pointer',
  fontFamily: 'monospace',
  fontSize: 12,
  fontWeight: 'bold',
  marginRight: 6,
  marginBottom: 4,
};

export const ControlPanel: React.FC<Props> = ({
  mode, onModeChange, waypoints, onClearWaypoints,
  onExecuteWaypoints, onRemoveWaypoint, onCancel, onSaveMap,
}) => {
  return (
    <div style={{
      position: 'absolute', bottom: 12, left: 12, right: 12, zIndex: 1000,
      background: 'rgba(10, 10, 30, 0.9)',
      color: '#e0e0e0',
      padding: '12px 16px',
      borderRadius: 8,
      fontFamily: 'monospace',
      fontSize: 13,
      boxShadow: '0 2px 12px rgba(0,0,0,0.5)',
    }}>
      <div style={{ display: 'flex', alignItems: 'center', flexWrap: 'wrap', gap: 4 }}>
        <button
          style={{
            ...btnStyle,
            background: mode === 'navigate' ? '#00ff41' : '#333',
            color: mode === 'navigate' ? '#000' : '#fff',
          }}
          onClick={() => onModeChange('navigate')}
        >
          Click-to-Navigate
        </button>
        <button
          style={{
            ...btnStyle,
            background: mode === 'waypoint' ? '#ffa500' : '#333',
            color: mode === 'waypoint' ? '#000' : '#fff',
          }}
          onClick={() => onModeChange('waypoint')}
        >
          Place Waypoints
        </button>
        <button
          style={{ ...btnStyle, background: '#e94560', color: '#fff' }}
          onClick={onCancel}
        >
          Stop
        </button>
        <button
          style={{ ...btnStyle, background: '#0f3460', color: '#fff' }}
          onClick={onSaveMap}
        >
          Save Map
        </button>
      </div>

      {waypoints.length > 0 && (
        <div style={{ marginTop: 8, borderTop: '1px solid #333', paddingTop: 8 }}>
          <div style={{ marginBottom: 4 }}>
            Waypoints ({waypoints.length}):
          </div>
          <div style={{ display: 'flex', gap: 4, flexWrap: 'wrap', marginBottom: 6 }}>
            {waypoints.map((wp, i) => (
              <span
                key={wp.id}
                style={{
                  background: '#ffa500',
                  color: '#000',
                  padding: '2px 8px',
                  borderRadius: 4,
                  cursor: 'pointer',
                  fontSize: 11,
                }}
                onClick={() => onRemoveWaypoint(wp.id)}
                title="Click to remove"
              >
                #{i + 1} ({wp.x.toFixed(1)}, {wp.y.toFixed(1)})
              </span>
            ))}
          </div>
          <button
            style={{ ...btnStyle, background: '#00ff41', color: '#000' }}
            onClick={onExecuteWaypoints}
          >
            Execute Route
          </button>
          <button
            style={{ ...btnStyle, background: '#555', color: '#fff' }}
            onClick={onClearWaypoints}
          >
            Clear All
          </button>
        </div>
      )}
    </div>
  );
};
