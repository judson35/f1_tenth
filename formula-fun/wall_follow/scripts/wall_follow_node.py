#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/ackermann_cmd'

        # create subscribers and publishers
        self.subscription = self.create_subscription(
                LaserScan,
                lidarscan_topic,
                self.scan_callback,
                10)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # set PID gains
        self.kp = 80.0
        self.ki = 0.0
        self.kd = 0.0

        # store history
        self.integral = 0.
        self.prev_error = 0.
        self.error = 0.

        # other stores
        self.lidar_range = 3*np.pi/2
        self.L = 0.25 # lookahead distance in meters
        self.t_last_scan = 0
        self.delT = 0. # time between scans and pid calls
        self.dist = 0.55 # desired distance to left wall (center of track to follow)
        
    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. 
        Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        # find the index of a in range_data to return range a
        angle_inc = self.lidar_range / len(range_data)
        phi = ((self.lidar_range / 2) - np.pi/2 + angle) # angle of arg 'angle' wrt 'angle_min'
        i = -round(phi / angle_inc)
        a = range_data[i]
        return a

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left
        (going counter clockwise in the Levine loop).
        You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        # find the angle of the vehicle to the wall, alpha using formula from repo
        b = self.get_range(range_data, 0)
        if type(b) != float:
            # wall cannot be found by lidar
            print('No wall detected. Please place vehicle on track')
            return 0.0

        else:
            # left wall found at range b from vehicle
            theta = 40
            i = 0
            alphas = []
            while theta <= 70:
                a = self.get_range(range_data, theta*np.pi/180)
                if type(a) == float:
                    alphas.append(np.arctan2(a*np.cos(theta) - b, a*np.sin(theta)))
                    i += 1
                theta += 1

            alpha = sum(alphas) / len(alphas)

            # use lookahead to find estimated distance to the left wall and compute error
            d_est = b*np.cos(alpha) + self.L*np.sin(alpha)
            error = dist - d_est
            return -error

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # Use kp, ki & kd to implement a PID controller
        p_ctrl = self.kp * error
        if self.delT != 0.:
            self.integral += (self.ki * error * self.delT)
            d_ctrl = self.kd * (error - self.prev_error) / self.delT
        else:
            d_ctrl = 0
        self.prev_error = error
        
        heading = p_ctrl + self.integral + d_ctrl
        
        # determine velocity based on heading angle
        if abs(heading) <= 20:
            if abs(heading) <= 10:
                velocity = 0.75
            else:
                velocity = 0.5
        else:
            velocity = 0.25

        velocity = velocity*2
  
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = heading * np.pi / 180 # convert to radians
        self.publisher_.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. 
        Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # determine time between last scan for integral and derivative gain
        if self.t_last_scan == 0:
            self.delT = 0.025 # avoid divide by zero
        else:
            self.delT = (msg.header.stamp.nanosec - self.t_last_scan) / 1e9
        error = self.get_error(msg.ranges, self.dist)
        self.pid_control(error) # actuate the car with PID
        self.t_last_scan = msg.header.stamp.nanosec


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
