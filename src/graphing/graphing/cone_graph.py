#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
import matplotlib.pyplot as plt
import numpy as np

class ConePlotter(Node):

    def __init__(self):
        super().__init__('cone_plotter')
        self.cone_subscription = self.create_subscription(
            PoseArray,
            'cone_location',
            self.cone_callback,
            10)
        self.desired_loc_subscription = self.create_subscription(
            Point,
            'desired_loc',
            self.desired_loc_callback,
            10)
        self.cone_subscription  # prevent unused variable warning
        self.desired_loc_subscription  # prevent unused variable warning

        self.desired_x = 0.0
        self.desired_y = 0.0

    def cone_callback(self, msg):
        x_values = []
        y_values = []
        for pose in msg.poses:
            x_values.append(pose.position.x)
            y_values.append(-1 * pose.position.y) #-1 is required on y values
        
        plt.clf()
        plt.axis([-50,50,-50,50])
        plt.scatter(y_values, x_values) #for better visualisation x & y are swapped
        plt.scatter(self.desired_y, self.desired_x)
        plt.pause(0.05)

    def desired_loc_callback(self, msg):
        self.desired_x = msg.x
        self.desired_y = -1 * msg.y
        

def main(args=None):
    rclpy.init(args=args)
    node = ConePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # close the figure on keyboard interrupt
        plt.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()