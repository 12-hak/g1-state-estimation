import React, { useRef, useEffect, useCallback, useState } from 'react';
import type { RobotPose, MapData, Waypoint } from '../types';

interface Props {
  pose: RobotPose | null;
  mapData: MapData | null;
  mapPoints: number[][];
  scanPoints: number[][];
  pointSize: number;
  waypoints: Waypoint[];
  onClickMap: (x: number, y: number) => void;
  onAddWaypoint: (x: number, y: number) => void;
  mode: 'navigate' | 'waypoint';
}

export const MapCanvas: React.FC<Props> = ({
  pose, mapData, mapPoints, scanPoints, pointSize, waypoints, onClickMap, onAddWaypoint, mode,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hasCenteredRef = useRef(false);
  const [view, setView] = useState({ offsetX: 0, offsetY: 0, scale: 80 });
  const dragRef = useRef({ dragging: false, didDrag: false, lastX: 0, lastY: 0 });

  const worldToCanvas = useCallback((wx: number, wy: number) => {
    return {
      cx: view.offsetX + wx * view.scale,
      cy: view.offsetY - wy * view.scale,
    };
  }, [view]);

  const canvasToWorld = useCallback((cx: number, cy: number) => {
    return {
      wx: (cx - view.offsetX) / view.scale,
      wy: -(cy - view.offsetY) / view.scale,
    };
  }, [view]);

  // Draw loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;

    // Background
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = '#16213e';
    ctx.lineWidth = 1;
    const gridStep = view.scale;
    const startX = view.offsetX % gridStep;
    const startY = view.offsetY % gridStep;
    for (let x = startX; x < w; x += gridStep) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = startY; y < h; y += gridStep) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }

    // North Indicator
    ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.font = '12px Inter';
    ctx.fillText('N', w / 2, 20);
    ctx.beginPath();
    ctx.moveTo(w / 2, 25);
    ctx.lineTo(w / 2, 5);
    ctx.stroke();

    // Map (occupancy grid) - HIDDEN to focus on high-fidelity LiDAR
    /*
    if (mapData) {
      const res = mapData.resolution;
      for (let gy = 0; gy < mapData.height; gy++) {
        for (let gx = 0; gx < mapData.width; gx++) {
          const val = mapData.data[gy * mapData.width + gx];
          if (val === -1) continue; // Unknown

          const wx = mapData.origin_x + gx * res + res / 2;
          const wy = mapData.origin_y + gy * res + res / 2;
          const { cx, cy } = worldToCanvas(wx, wy);

          const cellSize = Math.max(res * view.scale, 1);

          if (val === 100) {
            ctx.fillStyle = '#e94560';  // Occupied = red
          } else if (val === 0) {
            ctx.fillStyle = '#0f3460';  // Free = dark blue
          } else {
            continue;
          }
          ctx.fillRect(cx - cellSize / 2, cy - cellSize / 2, cellSize, cellSize);
        }
      }
    }
    */

    // Planned path
    if (mapData?.path && mapData.path.length > 1) {
      ctx.strokeStyle = '#00ff88';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 3]);
      ctx.beginPath();
      const start = worldToCanvas(mapData.path[0][0], mapData.path[0][1]);
      ctx.moveTo(start.cx, start.cy);
      for (let i = 1; i < mapData.path.length; i++) {
        const pt = worldToCanvas(mapData.path[i][0], mapData.path[i][1]);
        ctx.lineTo(pt.cx, pt.cy);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Waypoints
    waypoints.forEach((wp, idx) => {
      const { cx, cy } = worldToCanvas(wp.x, wp.y);
      ctx.fillStyle = '#ffa500';
      ctx.beginPath();
      ctx.arc(cx, cy, 8, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${idx + 1}`, cx, cy + 4);
    });

    // Accumulated world-frame map (blue -- stays fixed as robot moves)
    const mapPtSize = Math.max(pointSize, 2);
    ctx.fillStyle = 'rgba(30, 140, 255, 0.55)';
    for (let i = 0; i < mapPoints.length; i++) {
      const { cx, cy } = worldToCanvas(mapPoints[i][0], mapPoints[i][1]);
      ctx.fillRect(cx - mapPtSize / 2, cy - mapPtSize / 2, mapPtSize, mapPtSize);
    }

    // Current scan (bright green -- shows live LiDAR sweep)
    ctx.fillStyle = 'rgba(0, 255, 100, 0.8)';
    for (let i = 0; i < scanPoints.length; i++) {
      const { cx, cy } = worldToCanvas(scanPoints[i][0], scanPoints[i][1]);
      ctx.fillRect(cx - pointSize / 2, cy - pointSize / 2, pointSize, pointSize);
    }

    // Origin marker (start position reference for loop closure)
    {
      const { cx, cy } = worldToCanvas(0, 0);
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10); ctx.stroke();
      ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
      ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2 * Math.PI); ctx.fill();
    }

    // Robot
    if (pose) {
      const { cx, cy } = worldToCanvas(pose.x, pose.y);
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-pose.yaw);

      // Body circle
      ctx.fillStyle = '#00ff41';
      ctx.beginPath();
      ctx.arc(0, 0, 10, 0, 2 * Math.PI);
      ctx.fill();

      // Direction arrow
      ctx.fillStyle = '#00cc33';
      ctx.beginPath();
      ctx.moveTo(16, 0);
      ctx.lineTo(-4, -7);
      ctx.lineTo(-4, 7);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    }

  }, [pose, mapData, mapPoints, scanPoints, waypoints, view, worldToCanvas]);

  // Mouse handlers
  const handleMouseDown = (e: React.MouseEvent) => {
    dragRef.current = { dragging: true, didDrag: false, lastX: e.clientX, lastY: e.clientY };
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!dragRef.current.dragging) return;
    dragRef.current.didDrag = true;
    setView(v => ({
      ...v,
      offsetX: v.offsetX + (e.clientX - dragRef.current.lastX),
      offsetY: v.offsetY + (e.clientY - dragRef.current.lastY),
    }));
    dragRef.current.lastX = e.clientX;
    dragRef.current.lastY = e.clientY;
  };

  const handleMouseUp = () => {
    dragRef.current.dragging = false;
  };

  const handleWheel = (e: React.WheelEvent) => {
    const factor = e.deltaY > 0 ? 0.9 : 1.1;
    setView(v => ({ ...v, scale: Math.max(5, Math.min(500, v.scale * factor)) }));
  };

  const handleClick = (e: React.MouseEvent) => {
    if (dragRef.current.didDrag) {
      dragRef.current.didDrag = false;
      return;
    }
    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;
    const { wx, wy } = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
    if (mode === 'waypoint') {
      onAddWaypoint(wx, wy);
    } else {
      onClickMap(wx, wy);
    }
  };

  // Center on robot (once)
  useEffect(() => {
    if (pose && !hasCenteredRef.current) {
      hasCenteredRef.current = true;
      const canvas = canvasRef.current;
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      setView(v => ({
        ...v,
        offsetX: rect.width / 2 - pose.x * v.scale,
        offsetY: rect.height / 2 + pose.y * v.scale,
      }));
    }
  }, [pose]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%', cursor: mode === 'waypoint' ? 'crosshair' : 'grab' }}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
      onWheel={handleWheel}
      onClick={handleClick}
    />
  );
};
