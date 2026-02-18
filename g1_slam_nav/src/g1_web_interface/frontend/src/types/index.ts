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

export type WSMessage =
  | { type: 'pose'; x: number; y: number; z: number; yaw: number; status: SlamStatus }
  | { type: 'map' } & MapData;
