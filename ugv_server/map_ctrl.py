from base_ctrl import BaseController
from position_estimators import OdometryFuser, OdometryFuserMAG, DifferentialDriveEKF
import matplotlib.pyplot as plt
import numpy as np
import io
import time
import math
import os
import yaml

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

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
        self.pos_estimator = OdometryFuserMAG(wheelbase_m=f['map_config']['wheelbase_m'])
        # self.pos_estimator = DifferentialDriveEKF(wheelbase_m=f['map_config']['wheelbase_m'], dt=f['map_config']['dt'])
        self.base_ctrl = base_ctrl
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.orientation = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.go_to_target = False

    def update(self, data):
        if data is None:
            return self.pos_x, self.pos_y, self.orientation
        x_est, p = self.pos_estimator.process_data(data)
        self.pos_x = x_est[0]
        self.pos_y = x_est[1]
        self.orientation = x_est[2]

        if self.go_to_target:
            self.__move_to_target()

        print(f"Updated Position: x={x_est[0]:.3f} m, y={x_est[1]:.3f} m, theta={math.degrees(x_est[2]):.2f} deg.")
        return x_est[0], x_est[1], x_est[2]

    def get_position(self):
        return self.pos_x, self.pos_y, self.orientation
    
    def reset_position(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_estimator.reset_position()

    def stop(self):
        self.go_to_target = False
        self.base_ctrl.base_ros_speed_ctrl(0.0, 0.0)

    def go_to(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.go_to_target = True

    def __move_to_target(self):
        # Simple proportional controller to go to target
        error_x = self.target_x - self.pos_x
        error_y = self.target_y - self.pos_y
        distance = math.hypot(error_x, error_y)
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = angle_to_target - self.orientation
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize

        # Stop if close enough to target
        if distance < f['map_config']['position_tolerance']:
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
        if angle_error > angular_tolerance or angle_error < -angular_tolerance:
            # Rotate towards target
            angular_velocity = f['map_config']['kp_angular'] * angle_error
            max_angular_velocity = f['map_config']['max_angular_speed']  # rad/s
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

    
    