import math
import time
import numpy as np

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
        self.x_est = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.P = np.diag([0.1, 0.1, 0.1]) # Initial uncertainty


        self.theta_odom = 0.0 # Orientation derived purely from Odometry (rad)
        # Previous sensor readings for calculating deltas
        self.prev_t = 0
        self.prev_odl = 0.0
        self.prev_odr = 0.0

    def reset_position(self):
        """
        Resets the odometry x and y state to the origin.
        """
        self.x_est = np.array([0.0, 0.0, self.x_est[2]])

    def process_data(self, data):
        return self.update(data)

    def update(self, data):
        """
        Processes a new data packet and updates the robot's pose.
        :param data: The new dictionary of sensor readings.
        :return: A tuple of the new fused pose (x, y, theta).
        """
        # 1. Extract and Calculate Time Delta (dt)
        t = float(time.time())
        dt = (t - self.prev_t)

        if dt <= 0:
            return self.x_est, self.P

        # 2. Extract Wheel Odometry Data
        odl = data.get('odl', self.prev_odl)
        odr = data.get('odr', self.prev_odr)
        
        # Calculate distance deltas (assuming 'odl' and 'odr' are in meters)
        delta_d_l = odl - self.prev_odl
        delta_d_r = odr - self.prev_odr

        # 3. Dead Reckoning (Odometry Calculation)
        
        # Average distance traveled by the center of the robot
        delta_d = (delta_d_r + delta_d_l) / 2.0
        
        # Change in orientation based purely on wheel odometry
        delta_theta_odom = (delta_d_r - delta_d_l) / self.WHEELBASE
        
        # Update odometry-only orientation
        self.theta_odom += delta_theta_odom
        
        # Gyroscope Z-axis reading (Angular Velocity)
        gz = data.get('gz', 0.0)
        
        # Change in orientation based purely on Gyroscope
        delta_theta_gyro = gz * dt
        
        # 4. IMU Fusion for Orientation (Complementary Filter)
        # FUSE the two orientation sources (IMU is usually more reliable for instantaneous change)
        # Note: We fuse the *change* in angle, not the absolute angle.
        delta_theta_fused = (self.ALPHA * delta_theta_gyro) + ((1 - self.ALPHA) * delta_theta_odom)

        # Average heading used for calculating linear displacement
        theta_avg = self.x_est[2] + (delta_theta_fused / 2.0)

        # Calculate linear displacement in the global frame
        delta_x = delta_d * math.cos(theta_avg)
        delta_y = delta_d * math.sin(theta_avg)

        # Update position
        self.x_est[0] += delta_x
        self.x_est[1] += delta_y

        
        self.x_est[2] += delta_theta_fused
        
        # Normalize theta to be within -pi to pi
        self.x_est[2] = math.atan2(math.sin(self.x_est[2]), math.cos(self.x_est[2]))

        # 5. Store current readings for the next iteration
        self.prev_t = t
        self.prev_odl = odl
        self.prev_odr = odr

        # 6. Return the final fused pose
        return self.x_est, self.P

class OdometryFuserMAG:    
    def __init__(self, wheelbase_m):
        self.WHEELBASE = wheelbase_m

        # State variables
        self.x = 0.0          # Global X position (m)
        self.y = 0.0          # Global Y position (m)
        self.theta = 0.0      # Global Orientation (rad) - Fused
        self.theta_odom = 0.0 # Orientation derived purely from Odometry (rad)

        # 1. State Vector: x = [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # 2. Covariance Matrix (P): Represents the uncertainty of the state
        self.P = np.diag([0.1, 0.1, 0.1]) # Initial uncertainty

        # Previous sensor readings for calculating deltas
        self.prev_t = 0
        self.prev_odl = 0.0
        self.prev_odr = 0.0

    def reset_position(self):
        """
        Resets the odometry x and y state to the origin.
        """
        self.x_est = np.array([0.0, 0.0, self.x_est[2]])

    def process_data(self, data):
        return self.update(data)

    def update(self, data):
        # 1. Extract and Calculate Time Delta (dt)
        t = float(time.time())
        dt = (t - self.prev_t)

        if dt <= 0:
            return self.x_est, self.P

        # 2. Extract Wheel Odometry Data
        odl = data.get('odl', self.prev_odl)
        odr = data.get('odr', self.prev_odr)
        
        # Calculate distance deltas (assuming 'odl' and 'odr' are in meters)
        delta_d_l = odl - self.prev_odl
        delta_d_r = odr - self.prev_odr

        # 3. Dead Reckoning (Odometry Calculation)
        
        # Average distance traveled by the center of the robot
        delta_d = (delta_d_r + delta_d_l) / 2.0
        
        # Change in orientation based purely on wheel odometry
        delta_theta_odom = (delta_d_r - delta_d_l) / self.WHEELBASE
        
        # Update odometry-only orientation
        self.theta_odom += delta_theta_odom
        
        # Average heading used for calculating linear displacement
        theta_avg = self.x_est[2] + (delta_theta_odom / 2.0)

        # Calculate linear displacement in the global frame
        delta_x = delta_d * math.cos(theta_avg)
        delta_y = delta_d * math.sin(theta_avg)

        # Update position
        self.x_est[0] += delta_x
        self.x_est[1] += delta_y

        # 4. IMU Fusion for Orientation (Complementary Filter)
        # Change in orientation based purely on Magnetometer
        # Simplest Mag Heading: atan2(my, mx) - assumes level, calibrated.
        theta_mag = math.atan2(data.get('my', 0.0), data.get('mx', 1.0)) 
          
        self.x_est[2] = theta_mag
        
        # Normalize theta to be within -pi to pi
        self.x_est[2] = math.atan2(math.sin(self.x_est[2]), math.cos(self.x_est[2]))


        # 5. Store current readings for the next iteration
        self.prev_t = t
        self.prev_odl = odl
        self.prev_odr = odr

        # 6. Return the final fused pose
        return self.x_est, self.P

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
        self.prev_t = 0
    
    def reset_position(self):
        """
        Resets the odometry x and y state to the origin.
        """
        self.x_est = np.array([0.0, 0.0, self.x_est[2]])

    def _get_control_input(self, data):
        """
        Calculates control inputs (linear velocity nu, angular velocity omega) from odometry.
        """
        odl = data['odl']
        odr = data['odr']
        
        delta_d_l = odl - self.prev_odl
        delta_d_r = odr - self.prev_odr
        
        nu = (delta_d_l + delta_d_r) / (2.0 * self.DT)
        omega = (delta_d_r - delta_d_l) / (self.WHEELBASE * self.DT)
        
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
        t = float(time.time())
        dt_current = (t - self.prev_t)
        self.DT = dt_current
        
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
            
            self.prev_t = t

        return self.x_est, self.P

