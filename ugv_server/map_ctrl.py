from base_ctrl import BaseController
import matplotlib.pyplot as plt
import numpy as np
import io


class MapController():
    def __init__(self, base_ctrl: BaseController):
        self.base_ctrl = base_ctrl
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.orientation = 0.0

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
            lidar_x = int(lidar_distance * np.cos(lidar_angle) * 0.05) + 320
            lidar_y = int(lidar_distance * np.sin(lidar_angle) * 0.05) + 240
            lidar_points.append((lidar_x, lidar_y))

        
        fig, ax = plt.subplots()
        ax.set_xlim(-640, 640)
        ax.set_ylim(-480, 480)
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

    
    