#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class PointGraphPlotter(Node):

    def __init__(self):
        super().__init__('point_plotter')
        self.point_graph_subscription = self.create_subscription(
            PoseArray,
            'points',
            self.point_graph_callback,
            10)
        self.point_graph_subscription  # prevent unused variable warning
        print("point_graph node initialised")

    def point_graph_callback(self, msg):
        valid_x_values = []
        valid_y_values = []
        valid_z_values = []
        candidate_x_values = []
        candidate_y_values = []
        candidate_z_values = []
        for pose in msg.poses:
            valid_x_values.append(pose.position.x)
            valid_y_values.append(pose.position.y)
            valid_z_values.append(pose.position.z)
            if pose.orientation.z == 1:
                candidate_x_values.append(pose.position.x)
                candidate_y_values.append(pose.position.y)
                candidate_z_values.append(pose.position.z)

        # Create a colormap
        colormap = plt.cm.get_cmap('viridis')
        valid_colors = colormap(valid_z_values)
        candidate_colors = colormap(candidate_z_values)
        
        # Create a 3D scatter plot for valid_values
        fig = plt.figure()
        ax_valid = fig.add_subplot(121, projection='3d')
        scatter_valid = ax_valid.scatter(valid_x_values, valid_y_values, valid_z_values, c=valid_colors)    
        ax_valid.set_title('Valid Values')
        ax_valid.set_xlabel('X')
        ax_valid.set_ylabel('Y')
        ax_valid.set_zlabel('Z')
        ax_valid.view_init(elev=20, azim=180)
        ax_valid.set_xlim(0, 20)
        ax_valid.set_ylim(-25, 25)
        ax_valid.set_zlim(0, 2)
        cbar_valid = plt.colorbar(scatter_valid)
        cbar_valid.set_label('Z-coordinate')
        scatter_valid.set_clim(0, 0.1)

        # Create a 3D scatter plot for candidate_values
        ax_candidate = fig.add_subplot(122, projection='3d')
        scatter_candidate = ax_candidate.scatter(candidate_x_values, candidate_y_values, candidate_z_values, c=candidate_colors)    
        ax_candidate.set_title('Candidate Values')
        ax_candidate.set_xlabel('X')
        ax_candidate.set_ylabel('Y')
        ax_candidate.set_zlabel('Z')
        ax_candidate.view_init(elev=20, azim=180)
        ax_candidate.set_xlim(0, 20)
        ax_candidate.set_ylim(-25, 25)
        ax_candidate.set_zlim(0, 2)
        cbar_candidate = plt.colorbar(scatter_candidate)
        cbar_candidate.set_label('Z-coordinate')
        scatter_candidate.set_clim(0, 0.1)

        plt.tight_layout()  # Adjust layout for better spacing
        plt.show() 
        plt.pause(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = PointGraphPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # close the figure on keyboard interrupt
        plt.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
