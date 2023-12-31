#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
import matplotlib.pyplot as plt
import numpy as np

class MapPlotter(Node):

    def __init__(self):
        super().__init__('map_plotter')
        self.map_subscription = self.create_subscription(
            PoseArray,
            'map',
            self.map_callback,
            10)
        self.map_subscription  # prevent unused variable warning
        print("map node initialised")

    def map_callback(self, msg):
        x_values = []
        y_values = []
        for pose in msg.poses:
            x_values.append(pose.position.x)
            y_values.append(pose.position.y)

        vehicle_orientation = msg.poses[0].orientation.z
        # Convert orientation to angle in degrees
        angle_deg = np.degrees(vehicle_orientation)
        
        plt.clf()
        plt.axis([-150,150,-20,180])
        plt.scatter(x_values[1:], y_values[1:], c='blue')
        # plt.scatter(x_values[0],y_values[0], c='red')
        plt.quiver(x_values[0],y_values[0], 10 * np.cos(vehicle_orientation), 10 * np.sin(vehicle_orientation), color='red')
        plt.pause(0.05)        

def main(args=None):
    rclpy.init(args=args)
    node = MapPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # close the figure on keyboard interrupt
        plt.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()