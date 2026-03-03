/** Pose from Point-LIO /aft_mapped_to_init (or similar). */
export interface RobotPose {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

/** Minimal map message: trajectory (path so far) for display. */
export interface MapMessage {
  trajectory?: number[][];
}

/** For future navigation UI (ControlPanel). */
export interface Waypoint {
  id: string;
  x: number;
  y: number;
  yaw: number;
}

/** SLAM status (StatusPanel); optional in messages. */
export interface SlamStatus {
  mapping_active: boolean;
  localization_valid: boolean;
  map_points: number;
  loop_closures: number;
  quality: number;
}

/** Point as [x, y, z] or [x, y, z, intensity] for height coloring (AxisColor). */
export type Point = number[];

export function pointZ(p: Point): number {
  return typeof p[2] === 'number' ? p[2] : 0;
}

export type WSMessage =
  | { type: 'pose'; x: number; y: number; z: number; yaw: number }
  | { type: 'map'; trajectory?: number[][] }
  | { type: 'status'; scan_points: Point[] }
  | { type: 'map_cloud'; map_points: Point[] };
