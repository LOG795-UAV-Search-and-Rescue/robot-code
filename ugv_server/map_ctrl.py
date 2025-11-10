from base_ctrl import BaseController
import matplotlib.pyplot as plt
import numpy as np
import io
import time
import numpy as np
import math
import os
import yaml

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

class OdometryFuser:
    """
    Calculates the robot's position (x, y) and orientation (theta) 
    by fusing wheel odometry with gyroscope data ('gz') using a Complementary Filter.
    """
    
    def __init__(self, wheelbase_m, alpha=0.9):
        """
        Initializes the odometry calculator.
        :param wheelbase_m: The distance between the centers of the wheels (in meters).
        :param alpha: The weight given to the IMU (gyroscope) data (0.0 to 1.0).
        """
        self.WHEELBASE = wheelbase_m
        self.ALPHA = alpha  # IMU weight (e.g., 0.9 means 90% IMU, 10% Odometry)

        # State variables
        self.x = 0.0          # Global X position (m)
        self.y = 0.0          # Global Y position (m)
        self.theta = 0.0      # Global Orientation (rad) - Fused
        self.theta_odom = 0.0 # Orientation derived purely from Odometry (rad)

        # Previous sensor readings for calculating deltas
        self.prev_T = 0
        self.prev_odl = 0.0
        self.prev_odr = 0.0

    def update(self, data):
        """
        Processes a new data packet and updates the robot's pose.
        :param data: The new dictionary of sensor readings.
        :return: A tuple of the new fused pose (x, y, theta).
        """
        # 1. Extract and Calculate Time Delta (dt)
        T = float(time.time())
        dt = (T - self.prev_T)

        if dt <= 0:
            return self.x, self.y, self.theta

        # 2. Extract Wheel Odometry Data
        odl = data.get('odl', self.prev_odl)
        odr = data.get('odr', self.prev_odr)
        
        # Calculate distance deltas (assuming 'odl' and 'odr' are in meters)
        delta_d_L = odl - self.prev_odl
        delta_d_R = odr - self.prev_odr

        # 3. Dead Reckoning (Odometry Calculation)
        
        # Average distance traveled by the center of the robot
        delta_d = (delta_d_R + delta_d_L) / 2.0
        
        # Change in orientation based purely on wheel odometry
        delta_theta_odom = (delta_d_R - delta_d_L) / self.WHEELBASE
        
        # Update odometry-only orientation
        self.theta_odom += delta_theta_odom
        
        # Average heading used for calculating linear displacement
        theta_avg = self.theta + (delta_theta_odom / 2.0)

        # Calculate linear displacement in the global frame
        delta_x = delta_d * math.cos(theta_avg)
        delta_y = delta_d * math.sin(theta_avg)

        # Update position
        self.x += delta_x
        self.y += delta_y

        # 4. IMU Fusion for Orientation (Complementary Filter)
        
        # Gyroscope Z-axis reading (Angular Velocity)
        gz = data.get('gz', 0.0)
        
        # Change in orientation based purely on Gyroscope
        delta_theta_gyro = gz * dt
        
        # FUSE the two orientation sources (IMU is usually more reliable for instantaneous change)
        # Note: We fuse the *change* in angle, not the absolute angle.
        delta_theta_fused = (self.ALPHA * delta_theta_gyro) + ((1 - self.ALPHA) * delta_theta_odom)
        
        self.theta += delta_theta_fused
        
        # Normalize theta to be within -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


        # 5. Store current readings for the next iteration
        self.prev_T = T
        self.prev_odl = odl
        self.prev_odr = odr

        # 6. Return the final fused pose
        return self.x, self.y, self.theta

class DifferentialDriveEKF:
    """
    Extended Kalman Filter for a Differential Drive Robot fusing Odometry, Gyro, and Magnetometer.
    Uses simplified motion and measurement models for clarity.
    """
    def __init__(self, wheelbase_m, dt):
        self.WHEELBASE = wheelbase_m
        self.DT = dt  # Time step

        # 1. State Vector: x = [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # 2. Covariance Matrix (P): Represents the uncertainty of the state
        self.P = np.diag([0.1, 0.1, 0.1]) # Initial uncertainty

        # 3. Process Noise Covariance (Q): Uncertainty added by the motion model
        # Reflects uncertainty in linear velocity (nu) and angular velocity (omega)
        self.Q = np.diag([0.01**2, 0.01**2, np.deg2rad(1.0)**2]) # Q = diag([d_nu, d_nu, d_omega])

        # 4. Measurement Noise Covariance (R): Sensor uncertainty
        # z = [w_gyro, theta_mag]
        self.R = np.diag([np.deg2rad(0.5)**2, np.deg2rad(5.0)**2]) # R = diag([d_gyro_rate, d_mag_heading])

        # Store previous odometry for control input calculation
        self.prev_odl = 0.0
        self.prev_odr = 0.0
        self.prev_T = 0
    
    def _get_control_input(self, data):
        """
        Calculates control inputs (linear velocity nu, angular velocity omega) from odometry.
        """
        odl = data['odl']
        odr = data['odr']
        
        delta_d_L = odl - self.prev_odl
        delta_d_R = odr - self.prev_odr
        
        nu = (delta_d_L + delta_d_R) / (2.0 * self.DT)
        omega = (delta_d_R - delta_d_L) / (self.WHEELBASE * self.DT)
        
        self.prev_odl = odl
        self.prev_odr = odr
        
        return np.array([nu, omega])

    def predict(self, u):
        """
        EKF Prediction Step: x_k_bar = f(x_k-1, u_k), P_k_bar = F_k P_k-1 F_k^T + Q
        """
        nu, omega = u
        x, y, theta = self.x_est

        # Motion Model (f): Predict next state
        if abs(omega) < 1e-6: # Straight motion
            x_pred = x + nu * self.DT * math.cos(theta)
            y_pred = y + nu * self.DT * math.sin(theta)
            theta_pred = theta
        else: # Turning motion
            r = nu / omega # Turning radius
            x_pred = x - r * math.sin(theta) + r * math.sin(theta + omega * self.DT)
            y_pred = y + r * math.cos(theta) - r * math.cos(theta + omega * self.DT)
            theta_pred = theta + omega * self.DT

        self.x_est = np.array([x_pred, y_pred, theta_pred])

        # Jacobian of the motion model (F): Linearization around the predicted state
        F = np.eye(3)
        F[0, 2] = -nu * self.DT * math.sin(theta_pred)
        F[1, 2] = nu * self.DT * math.cos(theta_pred)

        # Predict Covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, z):
        """
        EKF Update Step: x_k = x_k_bar + K_k * (z_k - h(x_k_bar))
        """
        # Measurement Model (h): Converts state estimate into predicted measurement
        # z = [w_gyro, theta_mag]
        H = np.array([
            [0, 0, 1],  # Gyro measures angular velocity (omega), which is theta_dot.
            [0, 0, 1]   # Mag measures absolute heading (theta).
        ])
        
        # Predicted measurement (z_hat)
        # Note: The gyroscope reading (gz) is the control input 'omega' in the prediction step.
        # So, the predicted gyro rate is the commanded/estimated rate:
        z_hat = np.array([self._get_control_input(self.data_copy)[1], self.x_est[2]])

        # Innovation (y): Difference between actual and predicted measurement
        y = z - z_hat
        y[1] = math.atan2(math.sin(y[1]), math.cos(y[1])) # Normalize angle difference

        # Kalman Gain (K)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update State and Covariance
        self.x_est = self.x_est + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
        
        # Normalize theta after update
        self.x_est[2] = math.atan2(math.sin(self.x_est[2]), math.cos(self.x_est[2]))


    def process_data(self, data):
        """Main EKF loop: Predict -> Update"""
        self.data_copy = data # Store for use in update step
        T = data.get('T', self.prev_T)
        dt_current = (T - self.prev_T) / 1000.0
        
        if dt_current > 0:
            # 1. Prediction (using Odometry/Controls)
            u = self._get_control_input(data)
            self.predict(u)
            
            # 2. Update (using IMU measurements)
            omega_gyro = data.get('gz', u[1]) # Use gyro rate
            # Simplest Mag Heading: atan2(my, mx) - assumes level, calibrated.
            theta_mag = math.atan2(data.get('my', 0.0), data.get('mx', 1.0)) 
            z = np.array([omega_gyro, theta_mag])
            self.update(z)
            
            self.prev_T = T

        return self.x_est, self.P

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
        self.pos_estimator = OdometryFuser(wheelbase_m=f['map_config']['wheelbase_m'], alpha=f['map_config']['alpha'])
        self.base_ctrl = base_ctrl
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.orientation = 0.0

    def update(self, data):
        if data is None:
            return self.pos_x, self.pos_y, self.orientation
        x, y, theta = self.pos_estimator.update(data)
        self.pos_x = x
        self.pos_y = y
        self.orientation = theta
        print(f"Updated Position: x={x:.3f} m, y={y:.3f} m, theta={math.degrees(theta):.2f} deg. From data: {data}")
        return x, y, theta

    def get_position(self):
        return self.pos_x, self.pos_y, self.orientation

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
                 arrow_length * math.cos(self.orientation),
                 arrow_length * math.sin(self.orientation),
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
        lidar_points = []
        for lidar_angle, lidar_distance in zip(self.base_ctrl.rl.lidar_angles_show, self.base_ctrl.rl.lidar_distances_show):
            lidar_x = int(lidar_distance * np.cos(lidar_angle) * 0.05)
            lidar_y = int(lidar_distance * np.sin(lidar_angle) * 0.05) * -1
            lidar_points.append((lidar_x, lidar_y))

        
        fig, ax = plt.subplots()
        ax.set_xlim(-320, 320)
        ax.set_ylim(-240, 240)
        ax.set_title('Lidar Data')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.grid(True)
        for point in lidar_points:
            ax.plot(point[0], point[1], 'bo', markersize=2)

        buffer = io.BytesIO()
        fig.savefig(buffer, format='jpeg')
        plt.close(fig)
        buffer.seek(0)
        lidar_graph = buffer.read()
        buffer.close()

        return lidar_graph

    
    