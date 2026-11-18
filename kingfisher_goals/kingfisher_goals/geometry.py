"""2D helpers shared by the goal nodes: everything is on the water plane, z up."""

from math import atan2, cos, sin

import numpy as np


def yaw_of(q):
    """Return the yaw of a geometry_msgs/Quaternion."""
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Return an angle in [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


def heading_error(pose, xy):
    """Return the angle from the bow of a geometry_msgs/Pose to a point in pose's frame."""
    x, y = to_body(pose, xy)[0]
    return atan2(y, x)


def transform_xy(transform, xy):
    """Apply a geometry_msgs/TransformStamped to an (N, 2) array of points, in 2D."""
    t = transform.transform.translation
    yaw = yaw_of(transform.transform.rotation)
    c, s = np.cos(yaw), np.sin(yaw)
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return xy @ np.array([[c, s], [-s, c]]) + [t.x, t.y]


def from_body(pose, xy):
    """Return (N, 2) points from the body frame of a geometry_msgs/Pose in pose's frame."""
    yaw = yaw_of(pose.orientation)
    c, s = np.cos(yaw), np.sin(yaw)
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return xy @ np.array([[c, s], [-s, c]]) + [pose.position.x, pose.position.y]


def to_body(pose, xy):
    """Return (N, 2) points from pose's frame in the body frame of a geometry_msgs/Pose."""
    yaw = yaw_of(pose.orientation)
    c, s = np.cos(yaw), np.sin(yaw)
    d = np.asarray(xy, dtype=float).reshape(-1, 2) - [pose.position.x, pose.position.y]
    return d @ np.array([[c, -s], [s, c]])
