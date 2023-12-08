# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Utility functions for Kinematic Single Track MPC waypoint tracker

Author: Hongrui Zheng, Johannes Betz, Ahmad Amine
Last Modified: 12/27/22
"""
import math
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_euler

# @njit(cache=True)
def nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    projections = trajectory[:-1,:] + (t*diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

def getTrajectoryMarkerMessage(frame_id: str, ns: str, x_traj: list, y_traj: list, yaw_traj: list, color: ColorRGBA) -> Marker:
    marker = Marker()
    marker.type = 7
    marker.action = 0
    marker.header.frame_id = frame_id

    marker.ns = ns

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0

    for x, y in zip(x_traj, y_traj):
        point = Point()

        point.x = x
        point.y = y
        point.z = 0.0

        marker.points.append(point)
        marker.colors.append(color)

    marker.points.append(Point(x=2.0, y=2.0, z=0.0))
    marker.colors.append(ColorRGBA(r=0.0, g=0.0,b=1.0,a=1.0))

    marker.lifetime.sec = 1

    return marker

def getTrajectoryMarkerArrayMessage(frame_id: str, ns: str, x_traj: list, y_traj: list, yaw_traj: list, color: ColorRGBA) -> MarkerArray:
    waypoints = MarkerArray()
    i = 0
    for x,y,yaw in zip(x_traj, y_traj, yaw_traj):
        marker = Marker()

        marker.type = 0
        marker.action = 0
        marker.header.frame_id = "/map"

        marker.ns = "mpc_waypoints"
        marker.id = i
        i += 1

        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        angle = quaternion_from_euler(0.0, 0.0, yaw)

        marker.pose.orientation.x = angle[0]
        marker.pose.orientation.y = angle[1]
        marker.pose.orientation.z = angle[2]
        marker.pose.orientation.w = angle[3]

        waypoints.markers.append(marker)

    return waypoints