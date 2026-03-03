/**
 * RViz-style 2D view: same layer order and styling as point_lio.rviz.
 * - CloudMap (Laser_map): AxisColor, no decay, size 0.015 m
 * - CloudRegistered: <2 m from robot = fast refresh from last 2 scans; ≥2 m = trail (capped for speed).
 * - Path (/path): trajectory line
 * - Robot pose
 */

import React, { useRef, useEffect, useCallback, useState, useMemo } from 'react';
import type { RobotPose, Point } from '../types';
import { pointZ } from '../types';

const MAP_POINT_SIZE_M = 0.015;
const SCAN_POINT_SIZE_M = 0.004;
/** Points within this (m) of robot use last 2 scans only for fast refresh; beyond = trail. */
const NEAR_RADIUS_M = 2.0;
/** Voxel (m) for near refresh layer. */
const NEAR_VOXEL_M = 0.02;
/** Voxel size (m) for trail storage. */
const TRAIL_VOXEL_M = 0.03;
/** Max trail points drawn beyond NEAR_RADIUS_M to keep frame rate up. */
const FAR_TRAIL_DRAW_MAX = 25000;

/** RViz AxisColor: rainbow by height (blue low → red high). */
function heightToColor(z: number, zMin: number, zMax: number): string {
  if (zMax <= zMin) return 'rgba(200,200,200,0.85)';
  const t = Math.max(0, Math.min(1, (z - zMin) / (zMax - zMin)));
  const hue = (1 - t) * 240;
  return `hsla(${hue}, 85%, 55%, 0.85)`;
}

export interface ScanFrame {
  points: Point[];
  receivedAt: number;
}

interface Props {
  pose: RobotPose | null;
  mapPoints: Point[];
  scanFrames: ScanFrame[];
  trajectory: number[][];
  trailRef: React.MutableRefObject<Map<string, number[]>>;
  trailVersion: number;
  onTrailUpdate: () => void;
}

export const MapCanvas: React.FC<Props> = ({
  pose,
  mapPoints,
  scanFrames,
  trajectory,
  trailRef,
  trailVersion,
  onTrailUpdate,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hasCenteredRef = useRef(false);
  const [view, setView] = useState({ offsetX: 0, offsetY: 0, scale: 80, angle: 0 });
  const dragRef = useRef<{ pan: boolean; rotate: boolean; didDrag: boolean; lastX: number; lastY: number }>({ pan: false, rotate: false, didDrag: false, lastX: 0, lastY: 0 });

  const worldToCanvas = useCallback((wx: number, wy: number) => {
    const c = Math.cos(view.angle);
    const s = Math.sin(view.angle);
    const wxr = c * wx - s * wy;
    const wyr = s * wx + c * wy;
    return {
      cx: view.offsetX + wxr * view.scale,
      cy: view.offsetY - wyr * view.scale,
    };
  }, [view]);

  const canvasToWorld = useCallback((cx: number, cy: number) => {
    const wxr = (cx - view.offsetX) / view.scale;
    const wyr = -(cy - view.offsetY) / view.scale;
    const c = Math.cos(-view.angle);
    const s = Math.sin(-view.angle);
    return {
      wx: c * wxr - s * wyr,
      wy: s * wxr + c * wyr,
    };
  }, [view]);

  // Add every point we see to the trail (voxel key). No decay — display refreshes as new scans add points.
  useEffect(() => {
    const trail = trailRef.current;
    const v = TRAIL_VOXEL_M;
    const trailKey = (p: Point) => {
      const x = p.length >= 1 ? Number(p[0]) : 0;
      const y = p.length >= 2 ? Number(p[1]) : 0;
      const z = p.length >= 3 ? Number(p[2]) : 0;
      return `${Math.floor(x / v)} ${Math.floor(y / v)} ${Math.floor(z / v)}`;
    };
    let added = 0;
    for (const frame of scanFrames) {
      for (const p of frame.points) {
        if (p.length < 2) continue;
        const k = trailKey(p);
        if (!trail.has(k)) {
          trail.set(k, [...p]);
          added++;
        }
      }
    }
    if (added > 0) onTrailUpdate();
  }, [scanFrames, trailRef, onTrailUpdate]);

  // Near (<2 m): last 2 frames only, voxel — cheap, no full trail iteration. Re-runs when pose/frames change.
  const nearPoints = useMemo(() => {
    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r2 = NEAR_RADIUS_M * NEAR_RADIUS_M;
    const dist2 = (p: Point) => {
      const px = Number(p[0] ?? 0);
      const py = Number(p[1] ?? 0);
      return (px - rx) * (px - rx) + (py - ry) * (py - ry);
    };
    const vNear = NEAR_VOXEL_M;
    const nearKey = (p: Point) => {
      const x = p.length >= 1 ? Number(p[0]) : 0;
      const y = p.length >= 2 ? Number(p[1]) : 0;
      const z = p.length >= 3 ? Number(p[2]) : 0;
      return `${Math.floor(x / vNear)} ${Math.floor(y / vNear)} ${Math.floor(z / vNear)}`;
    };
    const seen = new Set<string>();
    const out: Point[] = [];
    for (const frame of scanFrames.slice(-2)) {
      for (const p of frame.points) {
        if (p.length < 2 || dist2(p) > r2) continue;
        const k = nearKey(p);
        if (seen.has(k)) continue;
        seen.add(k);
        out.push(p);
      }
    }
    return out;
  }, [scanFrames, pose?.x, pose?.y]);

  // Far: fixed set of trail voxels (sort keys, sample 25k). Only when trail changes — no sort on every pose.
  const farPointsSampled = useMemo(() => {
    const trail = trailRef.current;
    const keys = Array.from(trail.keys()).sort();
    if (keys.length === 0) return [];
    const n = Math.min(FAR_TRAIL_DRAW_MAX, keys.length);
    const step = keys.length / n;
    const out: Point[] = [];
    for (let i = 0; i < n; i++) {
      const k = keys[Math.min(Math.floor(i * step), keys.length - 1)];
      const p = trail.get(k);
      if (p && p.length >= 2) out.push(p);
    }
    return out;
  }, [trailRef, trailVersion]);

  // Global Z range for AxisColor (map + scan)
  const allPoints = useMemo(
    () => [...mapPoints, ...nearPoints, ...farPointsSampled],
    [mapPoints, nearPoints, farPointsSampled]
  );
  const { zMin, zMax } = useMemo(() => {
    if (allPoints.length === 0) return { zMin: 0, zMax: 1 };
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < allPoints.length; i++) {
      const z = pointZ(allPoints[i]);
      if (!Number.isNaN(z)) { lo = Math.min(lo, z); hi = Math.max(hi, z); }
    }
    if (hi <= lo) hi = lo + 1;
    return { zMin: lo, zMax: hi };
  }, [allPoints]);

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

    // Background — black like RViz
    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, w, h);

    // Grid (subtle)
    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 1;
    const gridStep = view.scale;
    let sx = view.offsetX % gridStep;
    if (sx < 0) sx += gridStep;
    let sy = view.offsetY % gridStep;
    if (sy < 0) sy += gridStep;
    for (let x = sx; x < w; x += gridStep) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = sy; y < h; y += gridStep) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }

    // 1) CloudMap (Laser_map): AxisColor, size 0.015 m, no decay. Cull off-screen.
    const mapPx = Math.max(1, MAP_POINT_SIZE_M * view.scale);
    const mapMargin = mapPx * 2;
    for (let i = 0; i < mapPoints.length; i++) {
      const p = mapPoints[i];
      if (p.length < 2) continue;
      const { cx, cy } = worldToCanvas(Number(p[0]), Number(p[1]));
      if (cx < -mapMargin || cy < -mapMargin || cx > w + mapMargin || cy > h + mapMargin) continue;
      ctx.fillStyle = heightToColor(pointZ(p), zMin, zMax);
      ctx.fillRect(cx - mapPx / 2, cy - mapPx / 2, mapPx, mapPx);
    }

    // 2) CloudRegistered: near then far (far only if ≥2 m so no double-draw). Cull off-screen.
    const scanPx = Math.max(1, SCAN_POINT_SIZE_M * view.scale);
    const margin = scanPx * 2;
    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r2 = NEAR_RADIUS_M * NEAR_RADIUS_M;
    const dist2 = (px: number, py: number) => (px - rx) * (px - rx) + (py - ry) * (py - ry);
    for (let i = 0; i < nearPoints.length; i++) {
      const p = nearPoints[i];
      if (p.length < 2) continue;
      const { cx, cy } = worldToCanvas(Number(p[0]), Number(p[1]));
      if (cx < -margin || cy < -margin || cx > w + margin || cy > h + margin) continue;
      ctx.fillStyle = heightToColor(pointZ(p), zMin, zMax);
      ctx.fillRect(cx - scanPx / 2, cy - scanPx / 2, scanPx, scanPx);
    }
    for (let i = 0; i < farPointsSampled.length; i++) {
      const p = farPointsSampled[i];
      if (p.length < 2) continue;
      const px = Number(p[0]);
      const py = Number(p[1]);
      if (dist2(px, py) < r2) continue;
      const { cx, cy } = worldToCanvas(px, py);
      if (cx < -margin || cy < -margin || cx > w + margin || cy > h + margin) continue;
      ctx.fillStyle = heightToColor(pointZ(p), zMin, zMax);
      ctx.fillRect(cx - scanPx / 2, cy - scanPx / 2, scanPx, scanPx);
    }

    // 3) Path (/path) — trajectory so far
    if (trajectory.length > 1) {
      ctx.strokeStyle = 'rgba(0, 200, 255, 0.9)';
      ctx.lineWidth = 2;
      ctx.setLineDash([]);
      ctx.beginPath();
      const t0 = worldToCanvas(trajectory[0][0], trajectory[0][1]);
      ctx.moveTo(t0.cx, t0.cy);
      for (let i = 1; i < trajectory.length; i++) {
        const pt = worldToCanvas(trajectory[i][0], trajectory[i][1]);
        ctx.lineTo(pt.cx, pt.cy);
      }
      ctx.stroke();
    }

    // Origin (camera_init)
    const orig = worldToCanvas(0, 0);
    ctx.strokeStyle = 'rgba(255,255,255,0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(orig.cx - 8, orig.cy); ctx.lineTo(orig.cx + 8, orig.cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(orig.cx, orig.cy - 8); ctx.lineTo(orig.cx, orig.cy + 8); ctx.stroke();

    // 4) Robot
    if (pose) {
      const { cx, cy } = worldToCanvas(pose.x, pose.y);
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-(pose.yaw + view.angle));
      ctx.fillStyle = '#00ff41';
      ctx.beginPath();
      ctx.arc(0, 0, 10, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillStyle = '#00cc33';
      ctx.beginPath();
      ctx.moveTo(14, 0);
      ctx.lineTo(-4, -6);
      ctx.lineTo(-4, 6);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }, [pose, mapPoints, nearPoints, farPointsSampled, trajectory, view, worldToCanvas, zMin, zMax]);

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    if (e.button === 0) dragRef.current = { ...dragRef.current, pan: true, lastX: e.clientX, lastY: e.clientY };
    if (e.button === 2) dragRef.current = { ...dragRef.current, rotate: true, didDrag: false, lastX: e.clientX, lastY: e.clientY };
  };
  const handleMouseMove = (e: React.MouseEvent) => {
    if (dragRef.current.pan) {
      dragRef.current.didDrag = true;
      setView(v => ({
        ...v,
        offsetX: v.offsetX + (e.clientX - dragRef.current.lastX),
        offsetY: v.offsetY + (e.clientY - dragRef.current.lastY),
      }));
    }
    if (dragRef.current.rotate) {
      dragRef.current.didDrag = true;
      const dx = e.clientX - dragRef.current.lastX;
      setView(v => ({ ...v, angle: v.angle + dx * 0.01 }));
    }
    dragRef.current.lastX = e.clientX;
    dragRef.current.lastY = e.clientY;
  };
  const handleMouseUp = (e: React.MouseEvent) => {
    if (e.button === 0) dragRef.current = { ...dragRef.current, pan: false };
    if (e.button === 2) dragRef.current = { ...dragRef.current, rotate: false };
  };
  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const factor = e.deltaY > 0 ? 0.9 : 1.1;
    setView(v => ({ ...v, scale: Math.max(5, Math.min(500, v.scale * factor)) }));
  };
  const handleContextMenu = (e: React.MouseEvent) => e.preventDefault();

  useEffect(() => {
    if (pose && !hasCenteredRef.current) {
      hasCenteredRef.current = true;
      const canvas = canvasRef.current;
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      setView(v => {
        const c = Math.cos(v.angle);
        const s = Math.sin(v.angle);
        const wxr = pose.x * c - pose.y * s;
        const wyr = pose.x * s + pose.y * c;
        return {
          ...v,
          offsetX: rect.width / 2 - wxr * v.scale,
          offsetY: rect.height / 2 + wyr * v.scale,
        };
      });
    }
  }, [pose]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%', display: 'block', cursor: 'grab' }}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
      onWheel={handleWheel}
      onContextMenu={handleContextMenu}
    />
  );
};
