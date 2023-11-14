#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

filename = "/home/judson35/f1_tenth/sim_ws/Race_2_Map_1_Waypoints.txt"

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        print("Initializing waypoint publisher")
        print(f"Waypoints Filename: {filename}")

        self.waypoint_publisher = self.create_publisher(Marker, 'target_waypoint', 10)
        self.timer = self.create_timer(0.5, self.timer_callback_)

        self.waypoints = []
        self.read_waypoint_file(filename)
        print("Publisher Initialized")

    def read_waypoint_file(self, filename):
        i = 0
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                if (i > 1):
                    data = line.split(',')
                    self.waypoints.append((float(data[0]), float(data[1])))
                    i = 0
                else:
                    i += 1

    def publish_waypoints(self):
        marker = Marker()

        marker.type = 7
        marker.action = 0
        marker.header.frame_id = "/map"

        marker.ns = "visualized_waypoints"

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        for waypoint in self.waypoints:
            point = Point()
            color = ColorRGBA()

            point.x = waypoint[0]
            point.y = waypoint[1]
            point.z = 0.0

            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
            color.a = 1.0

            marker.points.append(point)
            marker.colors.append(color)
        marker.lifetime.sec = 1
        self.waypoint_publisher.publish(marker)

    def timer_callback_(self):
        self.publish_waypoints()

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()