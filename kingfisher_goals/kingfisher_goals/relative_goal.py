"""
Runs a list of goals relative to the boat, keeping them inside a test area.

* the list comes from ``goals_file`` (a scenario YAML: ``goals: [[x, y], ...]``, in the robot
  frame), and each goal is placed from the boat's pose when it is selected;
* the next goal is the first remaining one inside ``area_half_size`` of ``area_frame``'s origin,
  in x and y. When none is, the boat heads for that origin until one is;
* a goal is sent once on ``goal``, in ``odom``'s frame. It ends reached, closer than
  ``reach_distance``, or missed, after ``goal_timeout``; either way it is done, ``goal/clear``
  stops the stream, and the next one is selected ``gap`` later;
* it runs while the policy has ``cmd_drive``: from ``control_button`` to ``teleop_button`` on
  ``joy``, the presses ``joy_teleop`` turns into ``drive_mux/select``. ``teleop_button`` puts the
  current goal back in the list;
* ``relative_goal/active`` (std_msgs/Bool, at ``rate``) is true while a goal is running,
  ``relative_goal/status`` (std_msgs/String, latched) says what it is doing, and
  ``relative_goal/markers`` (visualization_msgs/MarkerArray, latched) shows the remaining goals
  and the selected one, in red, in ``area_frame``.
"""

from math import atan2, degrees, hypot

from geometry_msgs.msg import Point, PoseStamped
from kingfisher_goals.geometry import from_body, transform_xy
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, ColorRGBA, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def marker(frame, marker_id, size, color, xy):
    """Return a sphere list of (N, 2) points at z = 0, or its deletion without points."""
    msg = Marker(ns='relative_goal', id=marker_id, type=Marker.SPHERE_LIST, color=color)
    msg.header.frame_id = frame
    msg.action = Marker.ADD if len(xy) else Marker.DELETE
    msg.pose.orientation.w = 1.0
    msg.scale.x = msg.scale.y = msg.scale.z = size
    msg.points = [Point(x=float(x), y=float(y)) for x, y in xy]
    return msg


class RelativeGoal(Node):
    """Send each goal of a list, relative to the boat, while it fits in the test area."""

    def __init__(self):
        super().__init__('relative_goal')

        goals_file = self.declare_parameter('goals_file', '').value
        rate = self.declare_parameter('rate', 10.0).value
        self._area_frame = self.declare_parameter('area_frame', 'test_center').value
        # m either side of area_frame's origin, in x and y
        self._half_size = self.declare_parameter('area_half_size', 15.0).value
        # m, a goal is reached closer than this
        self._reach_distance = self.declare_parameter('reach_distance', 0.33).value
        # s before a goal is missed
        self._goal_timeout = Duration(
            seconds=self.declare_parameter('goal_timeout', 30.0).value)
        # s between a goal ending and the next one
        self._gap = Duration(seconds=self.declare_parameter('gap', 5.0).value)
        # joy buttons of drive_mux's control/cmd_drive and teleop/cmd_drive
        self._control_button = self.declare_parameter('control_button', 0).value
        self._teleop_button = self.declare_parameter('teleop_button', 1).value

        with open(goals_file) as f:
            goals = yaml.safe_load(f)['goals']
        # (N, 2) in the robot frame, not done yet
        self._remaining = np.array(goals, dtype=float).reshape(-1, 2)
        self._total = len(self._remaining)
        self.get_logger().info(f'{self._total} goals from {goals_file}')

        self._odom = None
        self._control = False
        self._to_area = None
        # (x, y) of the area's origin in odom's frame
        self._center = None
        # (N, 2) the remaining goals in the area frame, when last computed
        self._area_xy = np.empty((0, 2))
        # index in the remaining goals (None without one), its (x, y) in odom's frame, its
        # start time and its description
        self._current = None
        self._target = None
        self._start = None
        self._label = None
        self._recentring = False
        self._gap_end = None
        self._status = None
        self._reached = 0
        self._missed = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self._clear_pub = self.create_publisher(Empty, 'goal/clear', 10)
        self._active_pub = self.create_publisher(Bool, 'relative_goal/active', 1)
        self._status_pub = self.create_publisher(String, 'relative_goal/status', latched)
        self._markers_pub = self.create_publisher(MarkerArray, 'relative_goal/markers', latched)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_timer(1.0 / rate, self._update)

    def _odom_callback(self, msg):
        self._odom = msg

    def _joy_callback(self, msg):
        if msg.buttons[self._teleop_button]:
            self._control = False
        elif msg.buttons[self._control_button]:
            self._control = True

    def _update(self):
        if len(self._remaining) and self._odom is not None and self._lookup_area():
            now = self.get_clock().now()
            if not self._control:
                self._release()
            elif self._current is not None:
                self._check_goal(now)
            elif self._gap_end is None or now >= self._gap_end:
                self._next_goal(now)
        self._active_pub.publish(Bool(data=self._current is not None))

    def _lookup_area(self):
        """Return whether the area's transforms are known, looking them up once."""
        if self._to_area is not None:
            return True
        frame = self._odom.header.frame_id
        try:
            to_area = self._tf_buffer.lookup_transform(self._area_frame, frame, Time())
            to_odom = self._tf_buffer.lookup_transform(frame, self._area_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No transform: {e}', throttle_duration_sec=1.0)
            return False
        self._to_area = to_area
        self._center = transform_xy(to_odom, [[0.0, 0.0]])[0]
        return True

    def _release(self):
        """Stop while the policy doesn't drive; the current goal stays in the list."""
        if self._current is not None:
            self.get_logger().info(f'Goal {self._label} put back: no control')
        if self._current is not None or self._recentring:
            self._end_goal()
        self._set_status('waiting for control')

    def _check_goal(self, now):
        p = self._odom.pose.pose.position
        elapsed = now - self._start
        reached = hypot(self._target[0] - p.x, self._target[1] - p.y) < self._reach_distance
        if not reached and elapsed < self._goal_timeout:
            return

        self._reached += reached
        self._missed += not reached
        self.get_logger().info(
            f'Goal {self._label} {"reached" if reached else "missed"} '
            f'after {elapsed.nanoseconds / 1e9:.1f} s')
        self._remaining = np.delete(self._remaining, self._current, axis=0)
        self._area_xy = np.delete(self._area_xy, self._current, axis=0)
        self._end_goal()
        self._gap_end = now + self._gap
        if len(self._remaining):
            self._set_status('gap')
        else:
            self.get_logger().info(f'Done: {self._reached} reached, {self._missed} missed')
            self._set_status('done')

    def _next_goal(self, now):
        xy = from_body(self._odom.pose.pose, self._remaining)
        self._area_xy = transform_xy(self._to_area, xy)
        fits = np.flatnonzero((np.abs(self._area_xy) <= self._half_size).all(axis=1))
        if len(fits) == 0:
            self._publish_markers()
            if not self._recentring:
                self.get_logger().info(f'No goal fits, heading for {self._area_frame}')
                self._recentring = True
                self._send(self._center)
                self._set_status('recentring')
            return

        self._recentring = False
        self._current = int(fits[0])
        self._target = xy[self._current]
        self._start = now
        x, y = self._remaining[self._current]
        self._label = (f'{self._reached + self._missed + 1}/{self._total} '
                       f'{hypot(x, y):.1f} m {degrees(atan2(y, x)):.0f} deg')
        twist = self._odom.twist.twist.linear
        self.get_logger().info(
            f'Goal {self._label} at ({self._target[0]:.2f}, {self._target[1]:.2f}), '
            f'boat at {hypot(twist.x, twist.y):.2f} m/s')
        self._send(self._target)
        self._publish_markers()
        self._set_status(self._label)

    def _end_goal(self):
        self._clear_pub.publish(Empty())
        self._current = None
        self._recentring = False
        self._publish_markers()

    def _send(self, xy):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._odom.header.frame_id
        goal.pose.position.x, goal.pose.position.y = float(xy[0]), float(xy[1])
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

    def _publish_markers(self):
        selected = self._area_xy[[self._current]] if self._current is not None else []
        self._markers_pub.publish(MarkerArray(markers=[
            marker(self._area_frame, 0, 0.3, ColorRGBA(g=0.8, b=0.8, a=0.8), self._area_xy),
            marker(self._area_frame, 1, 0.6, ColorRGBA(r=1.0, a=1.0), selected),
        ]))

    def _set_status(self, status):
        if status != self._status:
            self._status = status
            self._status_pub.publish(String(data=status))


def main(args=None):
    """Spin the relative goal node."""
    rclpy.init(args=args)
    node = RelativeGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
