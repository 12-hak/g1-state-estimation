/**
 * RViz-style 2D view:
 * - Trail (≥2 m): persistent 2D occupancy grid, self-corrects when robot revisits.
 * - Near (<2 m): point cloud from last 2 scans — priority, accurate, fast.
 * - Path, robot pose on top.
 */

import React, { useRef, useEffect, useCallback, useState, useMemo } from 'react';
import type { RobotPose, Point } from '../types';
import { pointZ } from '../types';

const SCAN_POINT_SIZE_M = 0.004;
const NEAR_RADIUS_M = 2.0;
const NEAR_VOXEL_M = 0.02;
/** 2D grid cell size (m) for trail occupancy. */
const GRID_CELL_M = 0.03;
/** Only points in this Z range become trail cells (skip ground/ceiling → walls and objects only). */
const TRAIL_Z_MIN = 0.1;
const TRAIL_Z_MAX = 2.0;
/** Trail cell colour — visible but not dominant. */
const TRAIL_COLOR = 'rgba(80, 140, 200, 0.7)';

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

// Persistent 2D occupancy grid outside React — survives remounts.
const occupancyGrid = new Set<string>();

// Offscreen canvas for trail: 1 pixel per cell, blit with one drawImage() per frame (O(1) draw).
const TRAIL_PIXEL = { r: 80, g: 140, b: 200, a: 0.7 };
let trailOffscreen: {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  minIx: number;
  maxIx: number;
  minIy: number;
  maxIy: number;
} | null = null;

function cellKey(x: number, y: number): string {
  return `${Math.floor(x / GRID_CELL_M)},${Math.floor(y / GRID_CELL_M)}`;
}
function cellCoords(key: string): [number, number] {
  const i = key.indexOf(',');
  return [Number(key.slice(0, i)), Number(key.slice(i + 1))];
}

function ensureTrailOffscreenBounds(ix: number, iy: number): void {
  if (!trailOffscreen) {
    const canvas = document.createElement('canvas');
    canvas.width = 1;
    canvas.height = 1;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    trailOffscreen = { canvas, ctx, minIx: ix, maxIx: ix, minIy: iy, maxIy: iy };
    ctx.fillStyle = `rgba(${TRAIL_PIXEL.r},${TRAIL_PIXEL.g},${TRAIL_PIXEL.b},${TRAIL_PIXEL.a})`;
    ctx.fillRect(0, 0, 1, 1);
    return;
  }
  const t = trailOffscreen;
  if (ix >= t.minIx && ix <= t.maxIx && iy >= t.minIy && iy <= t.maxIy) return;
  const newMinIx = Math.min(t.minIx, ix);
  const newMaxIx = Math.max(t.maxIx, ix);
  const newMinIy = Math.min(t.minIy, iy);
  const newMaxIy = Math.max(t.maxIy, iy);
  const w = newMaxIx - newMinIx + 1;
  const h = newMaxIy - newMinIy + 1;
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.fillStyle = `rgba(${TRAIL_PIXEL.r},${TRAIL_PIXEL.g},${TRAIL_PIXEL.b},${TRAIL_PIXEL.a})`;
  for (const key of occupancyGrid) {
    const [cix, ciy] = cellCoords(key);
    ctx.fillRect(cix - newMinIx, ciy - newMinIy, 1, 1);
  }
  trailOffscreen = { canvas, ctx, minIx: newMinIx, maxIx: newMaxIx, minIy: newMinIy, maxIy: newMaxIy };
}

function drawTrailCell(ix: number, iy: number): void {
  if (!trailOffscreen) {
    ensureTrailOffscreenBounds(ix, iy);
    return;
  }
  const t = trailOffscreen;
  if (ix < t.minIx || ix > t.maxIx || iy < t.minIy || iy > t.maxIy) {
    ensureTrailOffscreenBounds(ix, iy);
    return;
  }
  t.ctx.fillStyle = `rgba(${TRAIL_PIXEL.r},${TRAIL_PIXEL.g},${TRAIL_PIXEL.b},${TRAIL_PIXEL.a})`;
  t.ctx.fillRect(ix - t.minIx, iy - t.minIy, 1, 1);
}

function clearTrailCell(ix: number, iy: number): void {
  if (!trailOffscreen) return;
  const t = trailOffscreen;
  if (ix < t.minIx || ix > t.maxIx || iy < t.minIy || iy > t.maxIy) return;
  t.ctx.clearRect(ix - t.minIx, iy - t.minIy, 1, 1);
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
    return {
      cx: view.offsetX + (c * wx - s * wy) * view.scale,
      cy: view.offsetY - (s * wx + c * wy) * view.scale,
    };
  }, [view]);

  // Update occupancy grid from scan frames.
  // - Far from robot (≥2 m): add cells, never remove.
  // - Near robot (<2 m): clear old cells in that radius, re-stamp from current scan → self-corrects.
  useEffect(() => {
    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r = NEAR_RADIUS_M;
    const r2 = r * r;
    const cell = GRID_CELL_M;

    // Only wall-height points become trail cells (skip ground/ceiling)
    const isWallHeight = (p: Point) => {
      const z = p.length >= 3 ? Number(p[2]) : 0;
      return z >= TRAIL_Z_MIN && z <= TRAIL_Z_MAX;
    };

    // Collect cells that current scan occupies within near radius (wall-height only)
    const nearScanCells = new Set<string>();
    const lastFrames = scanFrames.slice(-2);
    for (const frame of lastFrames) {
      for (const p of frame.points) {
        if (p.length < 2 || !isWallHeight(p)) continue;
        const px = Number(p[0]);
        const py = Number(p[1]);
        const k = cellKey(px, py);
        const dx = px - rx, dy = py - ry;
        if (dx * dx + dy * dy <= r2) nearScanCells.add(k);
      }
    }

    // Clear grid cells within near radius that are NOT in current scan (self-correct)
    const nearIxMin = Math.floor((rx - r) / cell);
    const nearIxMax = Math.floor((rx + r) / cell);
    const nearIyMin = Math.floor((ry - r) / cell);
    const nearIyMax = Math.floor((ry + r) / cell);
    for (let ix = nearIxMin; ix <= nearIxMax; ix++) {
      for (let iy = nearIyMin; iy <= nearIyMax; iy++) {
        const cx = (ix + 0.5) * cell;
        const cy = (iy + 0.5) * cell;
        const dx = cx - rx, dy = cy - ry;
        if (dx * dx + dy * dy > r2) continue;
        const k = `${ix},${iy}`;
        if (!nearScanCells.has(k)) {
          occupancyGrid.delete(k);
          clearTrailCell(ix, iy);
        }
      }
    }

    // Add wall-height scan points to the grid
    let added = 0;
    for (const frame of scanFrames) {
      for (const p of frame.points) {
        if (p.length < 2 || !isWallHeight(p)) continue;
        const k = cellKey(Number(p[0]), Number(p[1]));
        if (!occupancyGrid.has(k)) {
          occupancyGrid.add(k);
          const [ix, iy] = cellCoords(k);
          drawTrailCell(ix, iy);
          added++;
        }
      }
    }

    // Also feed through to the 3D trail ref for near-scan voxel use
    const trail = trailRef.current;
    const v = 0.03; // TRAIL_VOXEL_M
    for (const frame of scanFrames) {
      for (const p of frame.points) {
        if (p.length < 2) continue;
        const x = p.length >= 1 ? Number(p[0]) : 0;
        const y = p.length >= 2 ? Number(p[1]) : 0;
        const z = p.length >= 3 ? Number(p[2]) : 0;
        const tk = `${Math.floor(x / v)} ${Math.floor(y / v)} ${Math.floor(z / v)}`;
        if (!trail.has(tk)) { trail.set(tk, [...p]); added++; }
      }
    }

    if (added > 0 || nearScanCells.size > 0) {
      onTrailUpdate();
    }
  }, [scanFrames, pose?.x, pose?.y, trailRef, onTrailUpdate]);

  // Near (<2 m): last 2 frames, voxel-downsampled — fast, accurate, priority.
  const nearPoints = useMemo(() => {
    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r2 = NEAR_RADIUS_M * NEAR_RADIUS_M;
    const seen = new Set<string>();
    const out: Point[] = [];
    for (const frame of scanFrames.slice(-2)) {
      for (const p of frame.points) {
        if (p.length < 2) continue;
        const px = Number(p[0]), py = Number(p[1]);
        if ((px - rx) * (px - rx) + (py - ry) * (py - ry) > r2) continue;
        const vn = NEAR_VOXEL_M;
        const k = `${Math.floor(px / vn)},${Math.floor(py / vn)},${Math.floor(Number(p[2] ?? 0) / vn)}`;
        if (seen.has(k)) continue;
        seen.add(k);
        out.push(p);
      }
    }
    return out;
  }, [scanFrames, pose?.x, pose?.y]);

  // Z range from near scan only (stable colour for close scan).
  const { zMin, zMax } = useMemo(() => {
    if (nearPoints.length === 0) return { zMin: 0, zMax: 1 };
    let lo = Infinity, hi = -Infinity;
    for (const p of nearPoints) {
      const z = pointZ(p);
      if (!Number.isNaN(z)) { lo = Math.min(lo, z); hi = Math.max(hi, z); }
    }
    if (hi <= lo) hi = lo + 1;
    return { zMin: lo, zMax: hi };
  }, [nearPoints]);

  // Main draw
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

    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 1;
    const gridStep = view.scale;
    let sx = view.offsetX % gridStep; if (sx < 0) sx += gridStep;
    let sy = view.offsetY % gridStep; if (sy < 0) sy += gridStep;
    for (let x = sx; x < w; x += gridStep) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
    for (let y = sy; y < h; y += gridStep) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }

    const rx = pose?.x ?? 0;
    const ry = pose?.y ?? 0;
    const r2 = NEAR_RADIUS_M * NEAR_RADIUS_M;

    // 1) Trail: single drawImage from offscreen canvas (1 px per cell); then punch hole for near radius.
    const t = trailOffscreen;
    if (t && t.canvas.width > 0 && t.canvas.height > 0) {
      const offW = t.canvas.width;
      const offH = t.canvas.height;
      ctx.save();
      ctx.translate(view.offsetX, view.offsetY);
      ctx.rotate(-view.angle);
      ctx.scale(view.scale, -view.scale);
      ctx.drawImage(
        t.canvas,
        0, 0, offW, offH,
        t.minIx * GRID_CELL_M, t.minIy * GRID_CELL_M,
        offW * GRID_CELL_M, offH * GRID_CELL_M
      );
      ctx.fillStyle = '#000000';
      ctx.beginPath();
      ctx.arc(rx, ry, NEAR_RADIUS_M, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
    }

    // 2) Near scan (<2 m): priority — height colour, drawn on top.
    const scanPx = Math.max(1, SCAN_POINT_SIZE_M * view.scale);
    const margin = scanPx * 2;
    for (let i = 0; i < nearPoints.length; i++) {
      const p = nearPoints[i];
      const { cx, cy } = worldToCanvas(Number(p[0]), Number(p[1]));
      if (cx < -margin || cy < -margin || cx > w + margin || cy > h + margin) continue;
      ctx.fillStyle = heightToColor(pointZ(p), zMin, zMax);
      ctx.fillRect(cx - scanPx / 2, cy - scanPx / 2, scanPx, scanPx);
    }

    // 3) Path
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

    // 4) Origin
    const orig = worldToCanvas(0, 0);
    ctx.strokeStyle = 'rgba(255,255,255,0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(orig.cx - 8, orig.cy); ctx.lineTo(orig.cx + 8, orig.cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(orig.cx, orig.cy - 8); ctx.lineTo(orig.cx, orig.cy + 8); ctx.stroke();

    // 5) Robot pose — top layer
    if (pose) {
      const { cx, cy } = worldToCanvas(pose.x, pose.y);
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-(pose.yaw + view.angle));
      ctx.fillStyle = '#00ff41';
      ctx.beginPath(); ctx.arc(0, 0, 10, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = '#00cc33';
      ctx.beginPath(); ctx.moveTo(14, 0); ctx.lineTo(-4, -6); ctx.lineTo(-4, 6); ctx.closePath(); ctx.fill();
      ctx.restore();
    }
  }, [pose, mapPoints, nearPoints, trajectory, view, worldToCanvas, zMin, zMax]);

  // Pan / zoom / rotate handlers
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
    setView(v => ({ ...v, scale: Math.max(5, Math.min(500, v.scale * (e.deltaY > 0 ? 0.9 : 1.1))) }));
  };

  useEffect(() => {
    if (pose && !hasCenteredRef.current) {
      hasCenteredRef.current = true;
      const canvas = canvasRef.current;
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      setView(v => {
        const c = Math.cos(v.angle), s = Math.sin(v.angle);
        return {
          ...v,
          offsetX: rect.width / 2 - (pose.x * c - pose.y * s) * v.scale,
          offsetY: rect.height / 2 + (pose.x * s + pose.y * c) * v.scale,
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
      onContextMenu={e => e.preventDefault()}
    />
  );
};
