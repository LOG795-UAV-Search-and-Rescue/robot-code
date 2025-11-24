"""Simple test harness for LidarPoseEstimator (ICP and PCA)
"""
from __future__ import annotations

import numpy as np
import math
import random
import sys
import os

this_dir = os.path.dirname(__file__)
proj_dir = os.path.dirname(this_dir)
sys.path.insert(0, proj_dir)

from lidar_pose import LidarPoseEstimator


def make_rectangular_map(width=4.0, height=2.0, spacing=0.05):
    # Create rectangle boundary map (points along the border)
    xs = np.arange(-width/2, width/2, spacing)
    ys = np.arange(-height/2, height/2, spacing)
    points = []
    for x in xs:
        points.append((x, -height/2))
        points.append((x, height/2))
    for y in ys:
        points.append((-width/2, y))
        points.append((width/2, y))
    return np.array(points)


def transform_points(pts, tx, ty, yaw, noise_sigma=0.0):
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    pts = (R @ pts.T).T + np.array([tx, ty])
    if noise_sigma > 0.0:
        pts += np.random.normal(scale=noise_sigma, size=pts.shape)
    return pts


def main():
    estimator = LidarPoseEstimator()

    # Create a reference map and a scan transformed by a known pose
    map_points = make_rectangular_map(4.5, 2.2, spacing=0.05)

    # Simulate robot: translate and rotate and take a scan from a subset of the map
    true_tx, true_ty, true_yaw = 1.2, -0.4, math.radians(17.5)

    # pick a neighborhood of the map as the scan (points within radius 3.0 of robot)
    full_map = map_points
    scan_simple = full_map[np.linalg.norm(full_map - np.array([0.0, 0.0]), axis=1) < 3.0][:200]
    scan = transform_points(scan_simple, true_tx, true_ty, true_yaw, noise_sigma=0.01)

    # Simulate an odometry prior that is slightly noisy / biased from true pose
    odo_est_x = true_tx + 0.05  # +5cm error
    odo_est_y = true_ty - 0.03  # -3cm error
    odo_est_yaw = true_yaw + math.radians(1.0)  # +1 deg error

    # Now estimate pose (align scan to the same map) using ICP with odometry prior
    tx, ty, yaw = estimator.estimate_pose_icp(scan, full_map, max_iters=100, tolerance=1e-6, max_match_distance=0.5,
                                               odometry_pose=(odo_est_x, odo_est_y, odo_est_yaw))

    print(f"True pose: tx={true_tx:.3f}, ty={true_ty:.3f}, yaw={math.degrees(true_yaw):.3f} deg")
    print(f"Estimated: tx={tx:.3f}, ty={ty:.3f}, yaw={math.degrees(yaw):.3f} deg")
    print(f"Errors: dx={(tx-true_tx):.3f}, dy={(ty-true_ty):.3f}, dyaw={(math.degrees(yaw-true_yaw)):.3f} deg")

    # Now PCA test when no map available
    # Create a line of points and verify PCA returns direction
    line_pts = np.array([[x, 0.02*random.random()] for x in np.linspace(-2.0, 2.0, 200)])
    yaw_est_pca = estimator.estimate_pose_pca(line_pts)
    print(f"PCA yaw (deg): {math.degrees(yaw_est_pca):.2f}")

    # --- Now mapping test: build a map incrementally from noisy odometry and scans ---
    print("\nMapping test: incremental map building")
    mapping_est = LidarPoseEstimator(map_resolution=0.02)
    # We'll simulate a robot tracing a path and receiving scans from the same rectangle map
    true_pose = [0.0, 0.0, 0.0]
    odom_pose = [0.0, 0.0, 0.0]
    # simulate 10 steps forward with small rotation
    for step in range(10):
        # advance true pose
        true_pose[0] += 0.2  # move 0.2m on x
        true_pose[2] += math.radians(2.0)  # small turn
        # pick subset of the map near true_pose
        local_scan = full_map[np.linalg.norm(full_map - np.array([true_pose[0], true_pose[1]]), axis=1) < 3.0][:200]
        scan = transform_points(local_scan, true_pose[0], true_pose[1], true_pose[2], noise_sigma=0.02)
        # odometry has drift
        odom_pose = (odom_pose[0] + 0.2 + random.normalvariate(0, 0.01), odom_pose[1] + random.normalvariate(0, 0.01), odom_pose[2] + math.radians(2.0) + math.radians(random.normalvariate(0, 0.5)))
        # register scan into map and get pose estimate in map frame
        estx, esty, estyaw = mapping_est.add_scan_to_map(scan, odometry_pose=odom_pose, fuse_with_odometry=True, odo_weight=0.6, max_match_distance=0.5, icp_max_iters=100)
        print(f"Step {step}: True ({true_pose[0]:.3f}, {true_pose[1]:.3f}, {math.degrees(true_pose[2]):.2f}) | Odo ({odom_pose[0]:.3f}, {odom_pose[1]:.3f}, {math.degrees(odom_pose[2]):.2f}) | Est ({estx:.3f}, {esty:.3f}, {math.degrees(estyaw):.2f})")
    print(f"Map points built: {mapping_est.get_map().shape[0]}")


if __name__ == '__main__':
    main()
