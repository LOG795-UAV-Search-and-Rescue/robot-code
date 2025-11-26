"""
Lidar pose estimation utilities

Provides simple PCA-based heading estimation and an ICP-based registration
method to align LIDAR scans to a reference map and compute robot pose
(translation and rotation).

Functions here are intentionally simple and dependency-free (uses NumPy only).
Suitable for small 2D scans. For large point clouds or production use, prefer
optimized libraries (e.g., Open3D, PCL, SciPy KDTree) and robust outlier
rejection.
"""

from __future__ import annotations

import numpy as np
import math
from typing import Tuple, Optional
try:
    from scipy.spatial import cKDTree as KDTree
except Exception:
    KDTree = None


class LidarPoseEstimator:
    """Estimator for 2D LIDAR scan poses.

    Methods:
    - estimate_pose_pca: Returns orientation (yaw) using PCA of scan points.
    - estimate_pose_icp: Iterative Closest Point to align a scan to a map
      and compute full pose (x, y, yaw) of the scan (robot) in map coordinates.

    Notes:
    - All points arrays should be Nx2 numpy arrays.
    - The ICP implementation provided is a straightforward, readable
      implementation with bruteforce nearest-neighbor matching. It is
      intentionally minimal but demonstrates the core algorithm.
    """

    def __init__(self, map_resolution: float = 0.02, max_map_points: int = 200000):
        """Create a LidarPoseEstimator which can optionally build a global
        point cloud map incrementally as scans are registered.

        Parameters:
        - map_resolution: resolution for simple downsampling/voxel merge (meters)
        - max_map_points: maximum retained points in the global map to bound memory
        """
        self.global_map = np.empty((0, 2), dtype=float)
        self.map_resolution = float(map_resolution)
        self.max_map_points = int(max_map_points)
        self._kdtree = None

    @staticmethod
    def estimate_pose_pca(points: np.ndarray) -> float:
        """Estimate heading (yaw) using PCA on 2D points.

        Arguments:
        - points: Nx2 array of (x, y) points (in robot frame or global frame).

        Returns:
        - yaw (float): angle in radians of the principal axis (range [-pi, pi]).

        This method returns the orientation of the dominant axis in the
        point cloud. It cannot provide absolute position (x, y) without
        an external reference (map or odometry).
        """
        if points is None or len(points) == 0:
            raise ValueError("Points must be a non-empty Nx2 numpy array")

        # Center the points
        pts = np.asarray(points, dtype=float)
        centroid = pts.mean(axis=0)
        pts_centered = pts - centroid

        # Covariance matrix
        cov = np.cov(pts_centered, rowvar=False)

        # Eigen decomposition
        eigvals, eigvecs = np.linalg.eigh(cov)

        # Principal eigenvector (largest eigenvalue)
        principal_idx = np.argmax(eigvals)
        principal_vec = eigvecs[:, principal_idx]

        # Yaw is the angle of the principal vector
        yaw = math.atan2(principal_vec[1], principal_vec[0])

        # Normalize yaw into [-pi, pi]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        return yaw

    @staticmethod
    def _closest_point_naive(src_pts: np.ndarray, dst_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """For each point in src_pts find closest point in dst_pts.

        Returns:
        - matched_src (Kx2), matched_dst (Kx2) arrays of matched pairs.
        """
        dst = np.asarray(dst_pts)
        src = np.asarray(src_pts)
        if len(dst) == 0 or len(src) == 0:
            return np.empty((0, 2)), np.empty((0, 2))

        matched_src = []
        matched_dst = []
        for s in src:
            # Compute squared distances to all dst points and pick min
            diffs = dst - s
            d2 = np.sum(diffs * diffs, axis=1)
            idx = int(np.argmin(d2))
            matched_src.append(s)
            matched_dst.append(dst[idx])

        return np.asarray(matched_src), np.asarray(matched_dst)

    def _build_kdtree(self):
        if KDTree is None:
            self._kdtree = None
            return
        if self.global_map.shape[0] == 0:
            self._kdtree = None
            return
        self._kdtree = KDTree(self.global_map)

    def _closest_point_kdtree(self, src_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self._kdtree is None:
            return self._closest_point_naive(src_pts, self.global_map)
        dists, idxs = self._kdtree.query(src_pts, k=1)
        matched_src = src_pts
        matched_dst = self.global_map[idxs]
        return matched_src, matched_dst

    @staticmethod
    def _best_fit_transform(A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """Given corresponding points A (N,2) and B (N,2) return rotation R (2x2)
        and translation t (2,) and yaw angle that maps A -> B in a least squares sense.
        Uses SVD-based Umeyama-like method.
        """
        assert A.shape[0] == B.shape[0]
        if A.shape[0] == 0:
            return np.eye(2), np.zeros(2), 0.0

        centroid_A = A.mean(axis=0)
        centroid_B = B.mean(axis=0)

        AA = A - centroid_A
        BB = B - centroid_B

        H = AA.T @ BB
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Reflection correction
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_B - R @ centroid_A
        yaw = math.atan2(R[1, 0], R[0, 0])
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        return R, t, yaw

    def estimate_pose_icp(self, scan: np.ndarray, ref_map: np.ndarray, max_iters: int = 50, tolerance: float = 1e-5, max_match_distance: Optional[float] = None, odometry_pose: Optional[Tuple[float, float, float]] = None, fuse_with_odometry: bool = False, odo_weight: float = 0.5) -> Tuple[float, float, float]:
        """Align `scan` to `ref_map` using Iterative Closest Point (ICP) and return
        the pose (x, y, yaw) of the scan in the `ref_map` coordinate frame.

        Parameters:
        - scan: Nx2 array of scan points (in scan/robot frame)
        - ref_map: Mx2 array of reference map points (in map frame)
        - max_iters: maximum ICP iterations
        - tolerance: stop when change in error is below this
        - max_match_distance: Optional max distance to consider matches; if
          specified, pairs with larger distance are discarded (helps reject
          outliers).

        Returns:
        - (x, y, yaw): translation and rotation that takes scan points into the
          map frame.

        If `ref_map` is None or empty, raises ValueError.
        """
        scan = np.asarray(scan, dtype=float)
        ref_map = np.asarray(ref_map, dtype=float)
        if scan.shape[0] == 0:
            raise ValueError("Scan must contain points")
        if ref_map.shape[0] == 0:
            raise ValueError("Reference map must contain points")

        # Initial transform: identity OR initialize based on odometry_pose
        if odometry_pose is not None:
            odo_x, odo_y, odo_yaw = odometry_pose
            R_total = np.array([[math.cos(odo_yaw), -math.sin(odo_yaw)], [math.sin(odo_yaw), math.cos(odo_yaw)]])
            t_total = np.array([odo_x, odo_y])
        else:
            R_total = np.eye(2)
            t_total = np.zeros(2)

        prev_error = float('inf')

        for i in range(max_iters):
            # Transform the scan points with current estimate
            transformed_scan = (R_total @ scan.T).T + t_total

            # Find correspondences (naive NN)
            # Use a KDTree based search if available for the reference map
            src_matched, dst_matched = self._closest_point_naive(transformed_scan, ref_map)

            # Optional distance threshold
            if max_match_distance is not None and len(src_matched) > 0:
                ds = np.linalg.norm(dst_matched - src_matched, axis=1)
                mask = ds <= float(max_match_distance)
                src_matched = src_matched[mask]
                dst_matched = dst_matched[mask]

            # Compute best fit transform from transformed_scan->ref_map
            if src_matched.shape[0] < 3:
                # Not enough matches to compute transform reliably; stop early
                break

            R_delta, t_delta, _ = self._best_fit_transform(src_matched, dst_matched)

            # Update total transform: new_T = [R_delta, t_delta] * [R_total, t_total]
            R_total = R_delta @ R_total
            t_total = R_delta @ t_total + t_delta

            # Compute mean error
            transformed_scan = (R_total @ scan.T).T + t_total
            # For error, compute distances for the matched src->dst points again
            src_matched2, dst_matched2 = self._closest_point_naive(transformed_scan, ref_map)
            if src_matched2.shape[0] == 0:
                break
            err = np.mean(np.linalg.norm(dst_matched2 - src_matched2, axis=1))

            print(prev_error)
            print(err)
            print(tolerance)
            print(abs(prev_error - err))
            print(abs(prev_error - err) < tolerance)

            if abs(prev_error - err) < tolerance:
                break
            prev_error = err

        yaw = math.atan2(R_total[1, 0], R_total[0, 0])
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        tx, ty = t_total[0], t_total[1]

        # Optionally fuse ICP result with odometry prior if provided. This gives
        # a simple convex fusion between the odometry pose and ICP pose. A more
        # sophisticated approach would use a Kalman filter or factor graph.
        if fuse_with_odometry and odometry_pose is not None:
            odo_x, odo_y, odo_yaw = odometry_pose
            w_icp = float(1.0 - odo_weight)
            w_odo = float(odo_weight)
            fused_x = w_icp * tx + w_odo * odo_x
            fused_y = w_icp * ty + w_odo * odo_y
            # Fuse angles using unit vector representation to avoid wrap issues
            s = w_icp * math.sin(yaw) + w_odo * math.sin(odo_yaw)
            c = w_icp * math.cos(yaw) + w_odo * math.cos(odo_yaw)
            fused_yaw = math.atan2(s, c)
            fused_yaw = (fused_yaw + math.pi) % (2 * math.pi) - math.pi
            return fused_x, fused_y, fused_yaw

        return tx, ty, yaw

    def reset_map(self):
        """Clear the internal global map."""
        self.global_map = np.empty((0, 2), dtype=float)
        self._kdtree = None

    def get_map(self) -> np.ndarray:
        return self.global_map.copy()

    @staticmethod
    def _downsample_points(points: np.ndarray, resolution: float) -> np.ndarray:
        """Simple voxel-grid downsampling: quantize points to a grid of `resolution`
        and keep unique voxels' centroids.
        """
        if points.shape[0] == 0:
            return points
        q = np.round(points / float(resolution))
        # Use structured unique trick to deduplicate rows
        q_view = np.ascontiguousarray(q).view(np.dtype((np.void, q.dtype.itemsize * q.shape[1])))
        _, idx = np.unique(q_view, return_index=True)
        uniq = points[idx]
        return uniq

    def add_scan_to_map(self, scan: np.ndarray, odometry_pose: Optional[Tuple[float, float, float]] = None, fuse_with_odometry: bool = False, odo_weight: float = 0.5, max_match_distance: Optional[float] = None, icp_max_iters: int = 50, icp_tolerance: float = 1e-5, downsample_resolution: Optional[float] = None) -> Tuple[float, float, float]:
        """Register a scan to the internal map. If the map is empty it will be
        initialized from the scan transformed by `odometry_pose` (or origin).

        Returns the estimated pose (x, y, yaw) of the scan in map coordinates.
        """
        scan = np.asarray(scan, dtype=float)
        if scan.shape[0] == 0:
            raise ValueError("Scan must contain points")

        # Use the odometry pose as initial guess and transform scan into map
        if odometry_pose is not None:
            odo_x, odo_y, odo_yaw = odometry_pose
            R_odo = np.array([[math.cos(odo_yaw), -math.sin(odo_yaw)], [math.sin(odo_yaw), math.cos(odo_yaw)]])
            transformed_scan = (R_odo @ scan.T).T + np.array([odo_x, odo_y])
        else:
            transformed_scan = scan.copy()

        if self.global_map.shape[0] == 0:
            # Initialize map from this scan
            pts = transformed_scan
            if downsample_resolution is None:
                downsample_resolution = self.map_resolution
            pts = self._downsample_points(pts, downsample_resolution)
            self.global_map = pts
            self._build_kdtree()
            # The estimated pose is the odometry_pose (or identity)
            if odometry_pose is not None:
                return odometry_pose
            else:
                return 0.0, 0.0, 0.0

        # Otherwise perform ICP with the current map as reference
        tx, ty, yaw = self.estimate_pose_icp(scan, self.global_map, max_iters=icp_max_iters, tolerance=icp_tolerance, max_match_distance=max_match_distance, odometry_pose=odometry_pose, fuse_with_odometry=fuse_with_odometry, odo_weight=odo_weight)

        # Transform the scan by the estimated pose into map frame and merge
        R_est = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        merged_scan = (R_est @ scan.T).T + np.array([tx, ty])

        # Downsample and merge with map
        if downsample_resolution is None:
            downsample_resolution = self.map_resolution
        merged = np.vstack((self.global_map, merged_scan))
        merged = LidarPoseEstimator._downsample_points(merged, downsample_resolution)
        # Ensure we cap the number of points to keep memory bounded
        if merged.shape[0] > self.max_map_points:
            # keep randomly sampled subset of points (preserving spread is better, but simple random is OK)
            idx = np.random.choice(merged.shape[0], self.max_map_points, replace=False)
            merged = merged[idx]

        self.global_map = merged
        self._build_kdtree()

        return tx, ty, yaw


# Small convenience wrappers
def estimate_pose_from_lidar(scan_points: np.ndarray, map_points: Optional[np.ndarray] = None, odometry_pose: Optional[Tuple[float, float, float]] = None, fuse_with_odometry: bool = False, odo_weight: float = 0.5) -> Tuple[float, float, float]:
    """Wrapper that tries to use ICP if map_points provided, otherwise uses PCA
    to return a yaw-only pose (x, y=0, yaw).
    """
    estimator = LidarPoseEstimator()
    if map_points is not None and len(map_points) > 0:
        return estimator.estimate_pose_icp(scan_points, map_points, odometry_pose=odometry_pose, fuse_with_odometry=fuse_with_odometry, odo_weight=odo_weight)
    else:
        yaw = estimator.estimate_pose_pca(scan_points)
        return 0.0, 0.0, yaw
