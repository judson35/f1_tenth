#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

filename = "waypoint_data.txt"

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')

        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 1)

    def clicked_point_callback(self, msg):
        with open(filename, 'a') as f:
            f.write(f"{msg.point.x},{msg.point.y}\n")

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()