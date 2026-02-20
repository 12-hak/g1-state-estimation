#!/usr/bin/env python3
"""
High-Speed Point Cloud to Wall Conversion Algorithms (NumPy-only version)

Implements fast algorithms for converting LiDAR point clouds into
geometric wall representations without external dependencies beyond NumPy.
Optimized for real-time performance on robot visualization.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class WallSegment:
    """Represents a detected wall segment."""
    points: np.ndarray  # Points belonging to this wall
    start: np.ndarray   # Start point (x, y, z)
    end: np.ndarray     # End point (x, y, z)
    normal: np.ndarray  # Wall normal vector
    thickness: float    # Wall thickness estimate
    confidence: float   # Detection confidence (0-1)


@dataclass
class WallPlane:
    """Represents a planar wall surface."""
    points: np.ndarray      # Points on this plane
    normal: np.ndarray      # Plane normal (a, b, c)
    distance: float         # Distance from origin (d in ax+by+cz=d)
    bounds: np.ndarray      # Bounding box [min_x, max_x, min_y, max_y, min_z, max_z]
    confidence: float       # Inlier ratio


class SimpleDBSCAN:
    """Simplified DBSCAN implementation using only NumPy."""
    
    def __init__(self, eps: float = 0.15, min_samples: int = 5):
        self.eps = eps
        self.min_samples = min_samples
    
    def fit(self, X: np.ndarray):
        """Cluster 2D points."""
        n = len(X)
        labels = -np.ones(n, dtype=int)  # -1 = noise
        cluster_id = 0
        
        for i in range(n):
            if labels[i] != -1:
                continue
            
            # Find neighbors
            distances = np.linalg.norm(X - X[i], axis=1)
            neighbors = np.where(distances <= self.eps)[0]
            
            if len(neighbors) < self.min_samples:
                continue  # Noise point
            
            # Start new cluster
            labels[i] = cluster_id
            
            # Expand cluster
            seed_set = list(neighbors)
            j = 0
            while j < len(seed_set):
                q = seed_set[j]
                if labels[q] == -1:
                    labels[q] = cluster_id
                elif labels[q] != -1:
                    j += 1
                    continue
                
                labels[q] = cluster_id
                
                # Find q's neighbors
                q_distances = np.linalg.norm(X - X[q], axis=1)
                q_neighbors = np.where(q_distances <= self.eps)[0]
                
                if len(q_neighbors) >= self.min_samples:
                    seed_set.extend([n for n in q_neighbors if n not in seed_set])
                
                j += 1
            
            cluster_id += 1
        
        self.labels_ = labels
        return self


class PointCloudToWalls:
    """High-speed point cloud to wall conversion (NumPy-only)."""
    
    def __init__(self, 
                 voxel_size: float = 0.05,
                 min_points_per_wall: int = 10,
                 ransac_threshold: float = 0.05,
                 dbscan_eps: float = 0.15,
                 dbscan_min_samples: int = 5):
        """
        Initialize wall detector.
        
        Args:
            voxel_size: Voxel grid size for downsampling (meters)
            min_points_per_wall: Minimum points to consider a wall
            ransac_threshold: RANSAC inlier threshold (meters)
            dbscan_eps: DBSCAN clustering radius (meters)
            dbscan_min_samples: DBSCAN minimum cluster size
        """
        self.voxel_size = voxel_size
        self.min_points_per_wall = min_points_per_wall
        self.ransac_threshold = ransac_threshold
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_samples = dbscan_min_samples
    
    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Fast voxel grid downsampling.
        
        Args:
            points: Nx3 array of points
            
        Returns:
            Downsampled points
        """
        if points.size == 0:
            return points
        
        # Quantize points to voxel grid
        voxel_coords = np.floor(points / self.voxel_size).astype(np.int32)
        
        # Find unique voxels
        unique_voxels, indices = np.unique(voxel_coords, axis=0, return_index=True)
        
        return points[indices]
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster points using simple DBSCAN.
        
        Args:
            points: Nx3 array of points
            
        Returns:
            List of point clusters
        """
        if len(points) < self.dbscan_min_samples:
            return [points] if len(points) > 0 else []
        
        # Use only X,Y for 2D clustering (walls are vertical)
        points_2d = points[:, :2]
        
        # Simple DBSCAN clustering
        clustering = SimpleDBSCAN(eps=self.dbscan_eps, 
                                 min_samples=self.dbscan_min_samples)
        clustering.fit(points_2d)
        
        labels = clustering.labels_
        
        # Extract clusters (ignore noise label -1)
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_mask = labels == label
            cluster_points = points[cluster_mask]
            if len(cluster_points) >= self.min_points_per_wall:
                clusters.append(cluster_points)
        
        return clusters
    
    def extract_line_segments(self, points: np.ndarray) -> List[WallSegment]:
        """
        Extract line segments from 2D points using PCA.
        
        Args:
            points: Nx3 array of points (uses X,Y only)
            
        Returns:
            List of wall segments
        """
        if len(points) < 2:
            return []
        
        # Project to 2D
        points_2d = points[:, :2]
        
        # Sort by angle from centroid for better line fitting
        centroid = np.mean(points_2d, axis=0)
        angles = np.arctan2(points_2d[:, 1] - centroid[1], 
                           points_2d[:, 0] - centroid[0])
        sorted_idx = np.argsort(angles)
        sorted_points = points[sorted_idx]
        sorted_2d = points_2d[sorted_idx]
        
        # Fit line using PCA
        centered = sorted_2d - centroid
        cov_matrix = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # Principal component (direction of maximum variance)
        principal_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, principal_idx].real
        
        # Project points onto line
        projections = np.dot(centered, direction)
        min_proj = np.min(projections)
        max_proj = np.max(projections)
        
        # Line endpoints
        start_2d = centroid + min_proj * direction
        end_2d = centroid + max_proj * direction
        
        # Use median Z height
        median_z = np.median(points[:, 2])
        start = np.array([start_2d[0], start_2d[1], median_z])
        end = np.array([end_2d[0], end_2d[1], median_z])
        
        # Normal perpendicular to line (in XY plane)
        normal_2d = np.array([-direction[1], direction[0]])
        normal = np.array([normal_2d[0], normal_2d[1], 0])
        
        # Estimate thickness
        distances_to_line = np.abs(np.dot(centered, normal_2d))
        thickness = np.percentile(distances_to_line, 90) * 2
        
        segment = WallSegment(
            points=sorted_points,
            start=start,
            end=end,
            normal=normal,
            thickness=max(thickness, 0.05),  # At least 5cm
            confidence=1.0
        )
        
        return [segment]
    
    def detect_walls_fast(self, points: np.ndarray) -> List[WallSegment]:
        """
        Fast wall detection pipeline.
        
        Pipeline:
        1. Voxel downsample for speed
        2. Cluster into separate structures
        3. Fit line segments to each cluster
        
        Args:
            points: Nx3 array of points
            
        Returns:
            List of detected wall segments
        """
        if points.size == 0:
            return []
        
        # Step 1: Downsample
        downsampled = self.voxel_downsample(points)
        
        if len(downsampled) < self.min_points_per_wall:
            return []
        
        # Step 2: Cluster
        clusters = self.cluster_points(downsampled)
        
        # Step 3: Fit segments to each cluster
        segments = []
        for cluster in clusters:
            cluster_segments = self.extract_line_segments(cluster)
            segments.extend(cluster_segments)
        
        return segments


class AdaptiveWallRenderer:
    """
    Adaptive wall rendering that combines point clouds with detected geometry.
    Optimized for real-time visualization.
    """
    
    def __init__(self):
        self.wall_detector = PointCloudToWalls(
            voxel_size=0.05,
            min_points_per_wall=15,
            ransac_threshold=0.05,
            dbscan_eps=0.2,
            dbscan_min_samples=8
        )
        self.cached_segments = []
        self.last_update_time = 0
        self.update_interval = 0.5  # Update geometry every 0.5s
    
    def update_walls(self, points: np.ndarray, current_time: float) -> List[WallSegment]:
        """
        Update wall geometry from point cloud.
        
        Args:
            points: Nx3 point cloud
            current_time: Current timestamp
            
        Returns:
            List of wall segments
        """
        # Only recompute periodically for performance
        if current_time - self.last_update_time < self.update_interval:
            return self.cached_segments
        
        self.cached_segments = self.wall_detector.detect_walls_fast(points)
        self.last_update_time = current_time
        
        return self.cached_segments
    
    def get_render_primitives(self, segments: List[WallSegment], 
                             wall_height: float = 1.5) -> List[dict]:
        """
        Convert wall segments to rendering primitives.
        
        Args:
            segments: List of wall segments
            wall_height: Height of walls
            
        Returns:
            List of render primitives (dicts with type, position, size, color)
        """
        primitives = []
        
        for segment in segments:
            # Calculate wall properties
            midpoint = (segment.start + segment.end) / 2
            direction = segment.end - segment.start
            length = np.linalg.norm(direction)
            
            if length < 0.01:
                continue
            
            # Create box primitive
            primitive = {
                'type': 'box',
                'position': midpoint + np.array([0, 0, wall_height/2]),
                'size': np.array([length/2, segment.thickness/2, wall_height/2]),
                'rotation': self._get_rotation_matrix(direction),
                'color': self._get_wall_color(segment.confidence),
                'alpha': 0.7 + 0.3 * segment.confidence
            }
            primitives.append(primitive)
        
        return primitives
    
    def _get_rotation_matrix(self, direction: np.ndarray) -> np.ndarray:
        """Compute rotation matrix to align box with direction."""
        direction_2d = direction[:2]
        length = np.linalg.norm(direction_2d)
        
        if length < 1e-6:
            return np.eye(3)
        
        direction_2d = direction_2d / length
        angle = np.arctan2(direction_2d[1], direction_2d[0])
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        return np.array([
            [cos_a, -sin_a, 0],
            [sin_a, cos_a, 0],
            [0, 0, 1]
        ])
    
    def _get_wall_color(self, confidence: float) -> np.ndarray:
        """Get color based on confidence (blue for uncertain, green for certain)."""
        # Interpolate from blue (low confidence) to green (high confidence)
        return np.array([
            0.0,
            confidence,
            1.0 - confidence,
            1.0
        ], dtype=np.float32)


# Convenience functions for quick usage
def quick_wall_detection(points: np.ndarray) -> List[WallSegment]:
    """Quick wall detection with default parameters."""
    detector = PointCloudToWalls()
    return detector.detect_walls_fast(points)
