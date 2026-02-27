import React, { useRef, useEffect } from 'react';
import type { RobotPose } from '../types';

interface Props {
    points: number[][];
    pointSize: number;
    pose: RobotPose | null;
}

export const LidarPanel: React.FC<Props> = ({ points, pointSize, pose }) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const w = canvas.width;
        const h = canvas.height;

        // Background
        ctx.fillStyle = 'rgba(26, 26, 46, 0.8)';
        ctx.fillRect(0, 0, w, h);

        // Crosshair
        ctx.strokeStyle = '#333';
        ctx.beginPath(); ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();

        // Range circles (5m, 10m, 15m)
        ctx.strokeStyle = '#444';
        ctx.setLineDash([2, 4]);
        const pxPerMeter = 15;
        [5, 10, 15].forEach(r => {
            ctx.beginPath();
            ctx.arc(w / 2, h / 2, r * pxPerMeter, 0, 2 * Math.PI);
            ctx.stroke();
        });
        ctx.setLineDash([]);

        // Points relative to robot
        if (pose) {
            ctx.fillStyle = '#00ff00';
            points.forEach(p => {
                // Transform world point to robot-relative
                const dx = p[0] - pose.x;
                const dy = p[1] - pose.y;

                // Rotate into robot local frame (optional, but top-down north-up is simpler here)
                const cx = w / 2 + dx * pxPerMeter;
                const cy = h / 2 - dy * pxPerMeter;

                if (cx > 0 && cx < w && cy > 0 && cy < h) {
                    ctx.fillRect(cx - pointSize / 2, cy - pointSize / 2, pointSize, pointSize);
                }
            });

            // Robot marker
            ctx.fillStyle = '#fff';
            ctx.beginPath();
            ctx.arc(w / 2, h / 2, 4, 0, 2 * Math.PI);
            ctx.fill();
        }
    }, [points, pointSize, pose]);

    return (
        <div style={{
            position: 'absolute',
            bottom: '20px',
            right: '250px',
            width: '200px',
            height: '200px',
            border: '2px solid #555',
            borderRadius: '8px',
            overflow: 'hidden',
            boxShadow: '0 4px 15px rgba(0,0,0,0.5)',
            pointerEvents: 'none',
            zIndex: 1000,
        }}>
            <div style={{
                position: 'absolute',
                top: 0, left: 0, width: '100%',
                background: 'rgba(0,0,0,0.6)',
                color: '#fff',
                fontSize: '10px',
                padding: '2px 6px',
                fontFamily: 'monospace',
            }}>
                RAW LIDAR VIEW (15m)
            </div>
            <canvas
                ref={canvasRef}
                width={200}
                height={200}
                style={{ width: '100%', height: '100%' }}
            />
        </div>
    );
};
