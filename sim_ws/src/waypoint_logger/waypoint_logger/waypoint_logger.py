#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

filename = "Race_2_Map_1_Waypoints.txt"

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        print(f"Filename: {filename}")

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 1)

        # self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 1)
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 1)
        self.timer = self.create_timer(0.1, self.timer_callback_)

        #Start car moving forward
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0
        self.drive_publisher.publish(msg)

        self.x = None
        self.y = None

    def timer_callback_(self):
        with open(filename, 'a') as f:
            f.write(f"{self.x},{self.y}\n")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()