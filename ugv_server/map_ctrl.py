from base_ctrl import BaseController
from position_estimators import OdometryEstimator, OdometryFuser, OdometryFuserMAG, DifferentialDriveEKF
from lidar_pose import LidarPoseEstimator
from typing import Tuple
import matplotlib.pyplot as plt
import numpy as np
import io
import time
import math
import os
import yaml
import sys

LINE_CLEAR = '\x1b[2K'

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

def print_replace(string):
    sys.stdout.write(LINE_CLEAR + '\r' + string)
    sys.stdout.flush()

# --- Example Usage ---
# WHEELBASE = 0.172  # meters
# DT = 0.05          # 50 ms time step (based on data T=1000 to T=1050)

# ekf = DifferentialDriveEKF(WHEELBASE, DT)

# # Example data packets (T is time in ms)
# data1 = {'T': 1000, 'L': 0, 'R': 0, 'gz': 0.0, 'mx': 50.0, 'my': 0.0, 'odl': 0.0, 'odr': 0.0, 'v': 0.0}
# data2 = {'T': 1050, 'L': 50, 'R': 50, 'gz': 0.1, 'mx': 49.0, 'my': 5.0, 'odl': 0.005, 'odr': 0.005, 'v': 0.1} # Straight move 5mm, slight rotation
# data3 = {'T': 1100, 'L': 30, 'R': 60, 'gz': 1.5, 'mx': 10.0, 'my': 40.0, 'odl': 0.006, 'odr': 0.007, 'v': 0.4} # Turning Left

# print("--- Extended Kalman Filter (EKF) Localization ---")

# # Data 1 (Initial Setup)
# ekf.process_data(data1)
# print(f"T={data1['T']}ms: Pose ({ekf.x_est[0]:.4f}, {ekf.x_est[1]:.4f}, {ekf.x_est[2]:.4f} rad)")

# # Data 2 (Prediction + Update)
# ekf.process_data(data2)
# print(f"T={data2['T']}ms: Pose ({ekf.x_est[0]:.4f}, {ekf.x_est[1]:.4f}, {ekf.x_est[2]:.4f} rad)")

# # Data 3 (Turning)
# ekf.process_data(data3)
# print(f"T={data3['T']}ms: Pose ({ekf.x_est[0]:.4f}, {ekf.x_est[1]:.4f}, {ekf.x_est[2]:.4f} rad)")


class MapController():
    def __init__(self, base_ctrl: BaseController):
        # self.pos_estimator = OdometryFuser(wheelbase_m=f['map_config']['wheelbase_m'], alpha=f['map_config']['alpha'])
        # self.pos_estimator = OdometryFuserMAG(wheelbase_m=f['map_config']['wheelbase_m'])
        # self.pos_estimator = DifferentialDriveEKF(wheelbase_m=f['map_config']['wheelbase_m'], dt=f['map_config']['dt'])
        self.base_ctrl = base_ctrl

        self.lidar_estimator = LidarPoseEstimator(map_resolution=f['map_config'].get('map_resolution_m', 0.02), max_map_points=f['map_config'].get('map_max_points', 200000))

        # get first position from base controller
        data = base_ctrl.feedback_data()
        while data is None or 'odl' not in data or 'odr' not in data:
            time.sleep(0.05)
            print(data)
            data = base_ctrl.feedback_data()
        self.odometry_start = (data['odl'], data['odr'])

        self.pos_estimator = OdometryEstimator(wheelbase_m=f['map_config']['wheelbase_m'], start_odl=self.odometry_start[0], start_odr=self.odometry_start[1])

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.go_to_target = False
        self.turn_to_target = False
        self.last_time = time.time()
        self.last_kpi_time = self.last_time
        self.kpi = []

    def update(self):
        data = self.base_ctrl.feedback_data()
        if data is None or 'odl' not in data or 'odr' not in data:
            return self.pos_x, self.pos_y, self.yaw
        
        # print(data)

        # If change exceeds max speed, ignore update
        t = time.time()
        dt = t - self.last_time
        self.last_time = t


        dt_kpi = t - self.last_kpi_time
        if dt_kpi >= 0.2:
            self.last_kpi_time = t
            self.kpi.append({'t': t, 'odl': data['odl'], 'odr': data['odr'], 'gx': data.get('gx', 0.0), 'gy': data.get('gy', 0.0), 'gz': data.get('gz', 0.0), 'mx': data.get('mx', 0.0), 'my': data.get('my', 0.0), 'mz': data.get('mz', 0.0), 'ax': data.get('ax', 0.0), 'ay': data.get('ay', 0.0), 'az': data.get('az', 0.0)})

        max_speed = f['map_config'].get('max_speed_m_s', 2.5)  # m/s
        max_distance = max_speed * dt
        odl_delta = data['odl'] - self.pos_estimator.prev_odl
        odr_delta = data['odr'] - self.pos_estimator.prev_odr
        if abs(odr_delta) > max_distance or abs(odl_delta) > max_distance:
            print(f"[WARN] Ignoring odometry update due to excessive speed: (L:{odl_delta/dt:.2f} m/s, R:{odr_delta/dt:.2f} m/s)")
            return self.pos_x, self.pos_y, self.yaw
        
        delta_x, delta_y, delta_yaw = self.pos_estimator.process_data(data['odl'], data['odr'])

        # delta_x, delta_y, delta_yaw = self.lidar_estimator.correct_pos_delta(self.get_lidar_points_m(), odometry_pose_delta=odometry_pose_delta)
        self.pos_x += delta_x
        self.pos_y += delta_y
        self.yaw += delta_yaw

        # Normalize yaw to [-pi, pi]
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        if self.go_to_target:
            self.__move_to_target()

        if self.turn_to_target:
            self.__turn_to_target()

        # print(f"Updated Position: x={self.pos_x:.3f} m, y={self.pos_y:.3f} m, theta={math.degrees(self.yaw):.2f} deg.")
        return self.pos_x, self.pos_y, self.yaw

    def get_position(self):
        return self.pos_x, self.pos_y, self.yaw
    
    def reset_position(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_estimator.reset_position()

    def reset_map(self):
        self.lidar_estimator.reset_map()

    def clear_map(self):
        """Alias to reset_map for clarity in higher level code."""
        self.reset_map()

    def stop(self):
        self.go_to_target = False
        self.base_ctrl.base_ros_speed_ctrl(0.0, 0.0)

    def go_to(self, target_x, target_y):
        print(f"Going to target: x={target_x} m, y={target_y} m")
        self.target_x = target_x
        self.target_y = target_y
        self.go_to_target = True

    def make_a_turn(self, angle_rad):
        self.target_yaw = self.yaw + angle_rad
        # Normalize target_yaw to [-pi, pi]
        self.target_yaw = (self.target_yaw + math.pi) % (2 * math.pi) - math.pi
        self.turn_to_target = True

    def __move_to_target(self):
        # Simple proportional controller to go to target
        error_x = self.target_x - self.pos_x
        error_y = self.target_y - self.pos_y
        distance = math.hypot(error_x, error_y)
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = angle_to_target - self.yaw
        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Stop if close enough to target
        if abs(distance) < f['map_config']['position_tolerance']:
            self.go_to_target = False
            self.base_ctrl.base_ros_speed_ctrl(0.0, 0.0)
            return

        # Control gains
        kp_linear = f['map_config']['kp_linear']
        kp_angular = f['map_config']['kp_angular']

        # Maximum velocities
        max_linear_velocity = f['map_config']['max_linear_speed']  # m/s
        max_angular_velocity = f['map_config']['max_angular_speed']  # rad/s

        angular_tolerance = f['map_config']['angular_tolerance']
        if abs(angle_error) > angular_tolerance:
            # Rotate towards target
            angular_velocity = kp_angular * angle_error
            max_angular_velocity = max_angular_velocity  # rad/s
            angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))
            self.base_ctrl.base_ros_speed_ctrl(0.0, angular_velocity)
            return
        

        # Calculate control commands
        linear_velocity = kp_linear * distance
        angular_velocity = kp_angular * angle_error

        # Limit velocities
        linear_velocity = max(-max_linear_velocity, min(max_linear_velocity, linear_velocity))
        angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))

        # Send commands to base controller
        self.base_ctrl.base_ros_speed_ctrl(linear_velocity, angular_velocity)

    def __turn_to_target(self):
        angle_error = self.target_yaw - self.yaw
        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Stop if close enough to target yaw
        if abs(angle_error) < 0.001:
            self.turn_to_target = False
            self.base_ctrl.base_ros_speed_ctrl(0.0, 0.0)
            return

        # Control gain
        kp_angular = f['map_config']['kp_angular']

        # Maximum angular velocity
        max_angular_velocity = f['map_config']['max_angular_speed']  # rad/s

        # Calculate control command
        angular_velocity = kp_angular * angle_error

        # Limit angular velocity
        angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))

        # Send command to base controller
        self.base_ctrl.base_ros_speed_ctrl(0.0, angular_velocity)

    def create_kpi_graph(self):
        """
        Creates a simple Matplotlib graph representing the KPI data.
        """
        times = [entry['t'] - self.kpi[0]['t'] for entry in self.kpi]
        odl_values = [entry['odl'] for entry in self.kpi]
        odr_values = [entry['odr'] for entry in self.kpi]
        rotation_values = [entry['gz'] for entry in self.kpi]
        acce_x_values = [entry['ax'] for entry in self.kpi]
        acce_y_values = [entry['ay'] for entry in self.kpi]
        acce_z_values = [entry['az'] for entry in self.kpi]

        fig, axs = plt.subplots(2, 2)
        axs[0, 0].plot(times, odl_values, label='Left Odometry (odl)', color='blue')
        axs[0, 0].plot(times, odr_values, label='Right Odometry (odr)', color='orange')
        axs[0, 0].set_title('KPI Odometry Data Over Time')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Odometry Readings (m)')
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        axs[0, 1].plot(times, rotation_values, label='Gyro Z (gz)', color='green')
        axs[0, 1].set_title('KPI Gyro Z Data Over Time')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Gyro Z (rad/s)')
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        axs[1, 0].plot(times, acce_x_values, label='Accel X (ax)', color='red')
        axs[1, 0].plot(times, acce_y_values, label='Accel Y (ay)', color='purple')
        axs[1, 0].plot(times, acce_z_values, label='Accel Z (az)', color='brown')
        axs[1, 0].set_title('KPI Accelerometer Data Over Time')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Acceleration (m/s²)')
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # estimated pose plot

        # Plot robot position
        axs[1, 1].set_xlim(-10, 10)
        axs[1, 1].set_ylim(-10, 10)
        axs[1, 1].set_title('Robot Pose')
        axs[1, 1].set_xlabel('X Position (m)')
        axs[1, 1].set_ylabel('Y Position (m)')

        axs[1, 1].plot(self.pos_x, self.pos_y, 'ro')  # Robot position
        # Indicate orientation with an arrow
        arrow_length = 0.5
        axs[1, 1].arrow(self.pos_x, self.pos_y,
                 arrow_length * math.cos(self.yaw),
                 arrow_length * math.sin(self.yaw),
                 head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        buffer = io.BytesIO()
        fig.savefig(buffer, format='jpeg')
        plt.close(fig)
        buffer.seek(0)
        img = buffer.read()
        buffer.close()

        return img


    def create_pose_graph(self):
        """
        Creates a simple Matplotlib graph representing the robot's pose.
        """
        fig, ax = plt.subplots()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_title('Robot Pose')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.grid(True)

        # Plot robot position
        ax.plot(self.pos_x, self.pos_y, 'ro')  # Robot position
        # Indicate orientation with an arrow
        arrow_length = 0.5
        ax.arrow(self.pos_x, self.pos_y,
                 arrow_length * math.cos(self.yaw),
                 arrow_length * math.sin(self.yaw),
                 head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        buffer = io.BytesIO()
        fig.savefig(buffer, format='jpeg')
        plt.close(fig)
        buffer.seek(0)
        img = buffer.read()
        buffer.close()

        return img

    def create_graph_as_bytes(self):
        """
        Creates a simple Matplotlib graph and returns the image data as bytes.
        """
        # 1. Prepare data
        x = np.linspace(0, 10, 100)
        y = np.sin(x)

        # 2. Create the plot
        fig, ax = plt.subplots()
        ax.plot(x, y, label='sin(x)', color='teal')
        ax.set_title('Sine Wave')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.legend()
        ax.grid(True)

        # 3. Save the figure to a BytesIO object (in-memory file)
        buffer = io.BytesIO()
        # Use fig.savefig to save the figure to the in-memory buffer
        # Specify the format (e.g., 'png', 'jpeg')
        fig.savefig(buffer, format='jpeg')
        
        # 4. Close the plot to free memory (important in non-interactive environments)
        plt.close(fig)

        # 5. Move the pointer to the start of the buffer
        buffer.seek(0)
        
        # 6. Read the content (the image bytes)
        image_bytes = buffer.read()
        
        # 7. Close the buffer
        buffer.close() 

        return image_bytes

    def lidar_frame_generate(self):
        # render lidar data
        lidar_points = self.get_lidar_points_m()

        fig, ax = plt.subplots()
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_title('Lidar Data')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.grid(True)
        for x, y in lidar_points:
            ax.plot(x, y, 'bo', markersize=1)
        # Plot current built map (if exists)
        map_pts = self.get_map_points()
        if map_pts is not None and map_pts.shape[0] > 0:
            # To keep within the same coordinate system centered at robot, convert
            for mx, my in map_pts:
                ax.plot(mx, my, 'r.', markersize=2)
        # Robot position marker at center
        ax.plot(0, 0, 'gx', markersize=6)

        buffer = io.BytesIO()
        fig.savefig(buffer, format='jpeg')
        plt.close(fig)
        buffer.seek(0)
        lidar_graph = buffer.read()
        buffer.close()

        return lidar_graph

    def get_lidar_points_m(self) -> np.ndarray:
        """Return lidar points in meters as an Nx2 numpy array.

        Uses the current contents of `self.base_ctrl.rl.lidar_angles_show` and
        `self.base_ctrl.rl.lidar_distances_show` and converts to (x, y) in meters
        relative to the robot frame (LIDAR origin).
        """
        angles = np.asarray(self.base_ctrl.rl.lidar_angles_show, dtype=float)
        distances = np.asarray(self.base_ctrl.rl.lidar_distances_show, dtype=float)
        if angles.size == 0 or distances.size == 0:
            return np.empty((0, 2))
        # distances in rl.lidar_distances_show are expected to be in meters
        distances_m = distances
        xs = distances_m * np.cos(angles)
        ys = distances_m * np.sin(angles)
        pts = np.column_stack((xs, ys))
        return pts

    def get_map_points(self) -> np.ndarray:
        return self.lidar_estimator.get_map()

    def estimate_pose_from_lidar(self, map_points: np.ndarray = None) -> Tuple[float, float, float]:
        """Estimate robot pose (x, y, yaw) using lidar scan.

        If `map_points` is provided, an ICP alignment will be performed and
        the pose is returned in the map coordinate frame. If no `map_points`
        are passed, then this will compute a PCA-based heading (yaw) and
        return (0, 0, yaw) because a global position cannot be inferred from
        a single scan without a reference map.
        """
        pts = self.get_lidar_points_m()
        if pts.shape[0] == 0:
            return 0.0, 0.0, 0.0

        # Use the internal estimator if no external map provided; otherwise
        # perform ICP against the supplied map.
        if map_points is not None and len(map_points) > 0:
            estimator = LidarPoseEstimator()
            odo_pose = (self.pos_x, self.pos_y, self.yaw)
            tx, ty, yaw = estimator.estimate_pose_icp(pts, map_points, odometry_pose=odo_pose)
            # Update internal pose to map-aligned pose
            self.pos_x = tx
            self.pos_y = ty
            self.yaw = yaw
            return tx, ty, yaw
        else:
            # Register scan into internal map (build the map incrementally)
            odo_pose = (self.pos_x, self.pos_y, self.yaw)
            tx, ty, yaw = self.lidar_estimator.add_scan_to_map(pts, odometry_pose=odo_pose, fuse_with_odometry=True, odo_weight=0.6, max_match_distance=f['map_config'].get('map_max_match_distance', 0.6), icp_max_iters=f['map_config'].get('icp_max_iters', 50), icp_tolerance=f['map_config'].get('icp_tolerance', 1e-5), downsample_resolution=f['map_config'].get('map_resolution_m', 0.02))
            # If no ICP occurred yet, fallback to PCA heading
            if tx is None:
                yaw = self.lidar_estimator.estimate_pose_pca(pts)
                tx, ty = self.pos_x, self.pos_y
            # Without map, we can't compute x/y in the map frame; keep previous
            # position or set to zero. We'll return a yaw with x/y = current
            # self.pos_x/self.pos_y so the caller can decide how to fuse.
            self.yaw = yaw
            return self.pos_x, self.pos_y, yaw

    
    