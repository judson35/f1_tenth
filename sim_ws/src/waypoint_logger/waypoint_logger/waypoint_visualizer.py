#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

filename = "waypoint_data.txt"

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.waypoint_publisher = self.create_publisher(Marker, 'target_waypoint', 100)

        self.waypoints = []

    def read_waypoint_file(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                data = line.split(',')
                self.waypoints.append((float(data[0]), float(data[1])))

    def publish_waypoints(self):
        marker_array = MarkerArray()
        for waypoint in self.waypoints:
            marker = Marker()
            marker.type = 2
            marker.action = 0
            marker.header.frame_id = "/map"

            marker.ns = "visualized_waypoints"

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]

            marker.lifetime.sec = 1

            # marker_array.markers.append(marker)
            # print(0)
            self.waypoint_publisher.publish(marker)
            time.sleep(1/10)
        # self.waypoint_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    waypoint_publisher.read_waypoint_file("waypoint_data.txt")
    waypoint_publisher.publish_waypoints()
    time.sleep(100)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()