#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        node.get_logger().info('Safety node')
        self.speed = 0.
        self.T = 1.3 # seconds, TTC threashold
        # create ROS subscribers and publishers.
        self.subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10)
        self.subscription = self.create_subscription(
                Odometry,
                '/ego_racecar/odom',
                self.odom_callback,
                10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        r_dot = np.array([], dtype = np.float32)
        TTC = np.array([], dtype = np.float32)
        for i in range(0, len(scan_msg.ranges)):
            theta = scan_msg.angle_min + i*scan_msg.angle_increment
            r_dot = np.append(r_dot, self.speed*np.cos(theta)) 
            if type(scan_msg.ranges[i]) == float:
                r = scan_msg.ranges[i]
                curr_r_dot = max(r_dot[i], 0.)
                if curr_r_dot != 0.:
                    # [x]_+ = max(x,0) ensures a postive range rate, r_dot
                    TTC = np.append(TTC, r / curr_r_dot)

        node.get_logger().info('Scan')
                   
        # publish command to brake
        if TTC.size > 0 and min(TTC) <= self.T:
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            self.publisher_.publish(ack_msg)
            node.get_logger().info('Stopping Car')
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
