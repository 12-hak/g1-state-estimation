export interface RobotPose {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export interface MapData {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
  data: number[];
  path?: number[][];
}

export interface SlamStatus {
  mapping_active: boolean;
  localization_valid: boolean;
  map_points: number;
  loop_closures: number;
  quality: number;
}

export interface Waypoint {
  id: string;
  x: number;
  y: number;
  yaw: number;
}

export interface DebugTelemetry {
  pose_seq: number;
  scan_seq: number;
  map_cloud_seq: number;
  pose_hz: number;
  scan_hz: number;
  map_cloud_hz: number;
  pose_age_ms: number;
  scan_age_ms: number;
  map_cloud_age_ms: number;
  ws_clients: number;
  scan_points: number;
  map_points: number;
  map_voxel_size: number;
  pose_yaw_deg?: number;
  pose_yaw_rate_deg_s?: number;
}

export type WSMessage =
  | { type: 'pose'; x: number; y: number; z: number; yaw: number; status: SlamStatus }
  | { type: 'map' } & MapData
  | { type: 'status'; scan_points: number[][]; point_size: number }
  | { type: 'map_cloud'; map_points: number[][] }
  | ({ type: 'debug' } & DebugTelemetry);
