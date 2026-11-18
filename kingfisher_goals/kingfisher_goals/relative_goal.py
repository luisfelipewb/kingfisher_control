"""
Runs a list of goals relative to the boat, keeping them inside a test area.

::

    waiting for control ─A─► lead-in ─ready─► goal ─reached/missed─► lead-in … ► done
                               │                           B from any state ─► waiting
                  no goal fits ▼
                           recentring ─ready─► goal

* the list comes from ``goals_file`` (a scenario YAML: ``goals: [[x, y], ...]``, in the robot
  frame). A goal fits when, placed from the boat's live pose, it is inside ``area_half_size`` of
  ``area_frame``'s origin in x and y;
* lead-in: the node drives the boat itself on ``control/cmd_drive``, holding the heading it had,
  at ``start_speed``. Recentring is the same, heading for the area's origin. The first goal in the
  file that fits is sent once the boat has been ready for ``settle_time``: within
  ``start_speed_tolerance`` of ``start_speed``, turning slower than ``start_yaw_rate`` and, when
  recentring, pointing at the origin within ``recentre_heading``;
* goal: sent once on ``goal``, in ``odom``'s frame, for the policy. It ends reached, closer than
  ``reach_distance``, or missed, after ``goal_timeout``; either way it is done, and ``goal/clear``
  stops the stream, and with it the policy;
* it runs from ``control_button`` to ``teleop_button`` on ``joy``, the presses ``joy_teleop`` turns
  into ``drive_mux/select``. ``teleop_button`` puts the current goal back in the list;
* ``relative_goal/active`` (std_msgs/Bool, at ``rate``) is true while a goal is running,
  ``relative_goal/status`` (std_msgs/String, latched) is the state, with why a lead-in waits or the
  goal's label, and ``relative_goal/markers`` (visualization_msgs/MarkerArray, latched) shows the
  remaining goals and the selected one, in red, in ``area_frame``.
"""

from dataclasses import fields
from math import atan2, cos, degrees, hypot, radians

from geometry_msgs.msg import Point, PoseStamped
from kingfisher_goals.geometry import from_body, heading_error, transform_xy, wrap, yaw_of
from kingfisher_goals.lead_in import LeadIn
from kingfisher_msgs.msg import Drive
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

WAITING = 'waiting for control'
LEAD_IN = 'lead-in'
RECENTRING = 'recentring'
GOAL = 'goal'
DONE = 'done'


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
    """Send each goal of a list, relative to the boat, from a steady lead-in."""

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
        # m/s forward and rad/s, the boat's state when a goal is sent
        self._start_speed = self.declare_parameter('start_speed', 0.5).value
        self._speed_tolerance = self.declare_parameter('start_speed_tolerance', 0.05).value
        self._start_yaw_rate = self.declare_parameter('start_yaw_rate', 0.05).value
        # m/s, the least speed while the bow is far off: the boat barely turns standing still
        self._turn_speed = self.declare_parameter('turn_speed', 0.2).value
        # deg between the bow and the area's origin, to end recentring
        self._recentre_heading = radians(self.declare_parameter('recentre_heading', 10.0).value)
        # s ready before a goal is sent; longer than the agent's episode_gap, so each goal starts
        # fresh
        self._settle_time = Duration(seconds=self.declare_parameter('settle_time', 1.0).value)
        self._lead_in = LeadIn(**{
            gain.name: self.declare_parameter(f'lead_in.{gain.name}', gain.default).value
            for gain in fields(LeadIn)})
        self._dt = 1.0 / rate
        # joy buttons of drive_mux's control/cmd_drive and teleop/cmd_drive
        self._control_button = self.declare_parameter('control_button', 0).value
        self._teleop_button = self.declare_parameter('teleop_button', 1).value

        with open(goals_file) as f:
            goals = yaml.safe_load(f)['goals']
        # (N, 2) in the robot frame, not done yet
        self._remaining = np.array(goals, dtype=float).reshape(-1, 2)
        self._total = len(self._remaining)
        self.get_logger().info(f'{self._total} goals from {goals_file}')

        self._state = None
        self._status = None
        self._odom = None
        self._control = False
        self._to_area = None
        # (x, y) of the area's origin in odom's frame
        self._center = None
        # (N, 2) the remaining goals in the area frame, when last placed
        self._area_xy = np.empty((0, 2))
        # lead-in: the heading to hold, in odom's frame, and since when the boat is ready
        self._heading = 0.0
        self._ready_since = None
        # goal: its index in the remaining goals, (x, y) in odom's frame, start time and label
        self._current = None
        self._target = None
        self._start = None
        self._label = None
        self._reached = 0
        self._missed = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self._clear_pub = self.create_publisher(Empty, 'goal/clear', 10)
        self._drive_pub = self.create_publisher(Drive, 'control/cmd_drive', 1)
        self._active_pub = self.create_publisher(Bool, 'relative_goal/active', 1)
        self._status_pub = self.create_publisher(String, 'relative_goal/status', latched)
        self._markers_pub = self.create_publisher(MarkerArray, 'relative_goal/markers', latched)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_timer(self._dt, self._update)
        self._set_state(WAITING if self._total else DONE)

    def _odom_callback(self, msg):
        self._odom = msg

    def _joy_callback(self, msg):
        if msg.buttons[self._teleop_button]:
            self._control = False
        elif msg.buttons[self._control_button]:
            self._control = True

    # States

    def _update(self):
        if self._state != DONE and self._odom is not None and self._lookup_area():
            now = self.get_clock().now()
            if not self._control:
                self._release()
            elif self._state == WAITING:
                self._start_lead_in()
            elif self._state == GOAL:
                self._on_goal(now)
            else:
                self._on_lead_in(now)
        self._active_pub.publish(Bool(data=self._state == GOAL))

    def _release(self):
        """Stop while the node doesn't have cmd_drive; the current goal stays in the list."""
        if self._state == GOAL:
            self.get_logger().info(f'Goal {self._label} put back: no control')
            self._clear_goal()
        elif self._state in (LEAD_IN, RECENTRING):
            self.get_logger().info(f'{self._state} stopped: no control')
        self._set_state(WAITING)

    def _start_lead_in(self, state=LEAD_IN):
        if state == RECENTRING:
            self.get_logger().info(f'No goal fits, heading for {self._area_frame}')
        self._heading = yaw_of(self._odom.pose.pose.orientation)
        self._ready_since = None
        self._lead_in.reset(self._odom.twist.twist.linear.x)
        self._set_state(state)

    def _on_lead_in(self, now):
        fits = self._fitting()
        if self._state == LEAD_IN and not len(fits):
            self._start_lead_in(RECENTRING)

        error = self._heading_error()
        reason = self._unready(fits, error)
        if reason is not None:
            self._ready_since = None
        elif self._ready_since is None:
            self._ready_since = now
        elif now - self._ready_since >= self._settle_time:
            self._start_goal(fits[0], now, error)
            return
        status = f'{self._state}: {reason or "settling"}'
        self._set_status(status)
        self.get_logger().info(status, throttle_duration_sec=5.0)
        self._drive(error)

    def _start_goal(self, index, now, error):
        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        aligned = (f', {degrees(error):.1f} deg off {self._area_frame}'
                   if self._state == RECENTRING else '')
        self._current = int(index)
        self._target = from_body(pose, self._remaining[self._current])[0]
        self._start = now
        x, y = self._remaining[self._current]
        self._label = (f'{self._reached + self._missed + 1}/{self._total} '
                       f'{hypot(x, y):.1f} m {degrees(atan2(y, x)):.0f} deg')
        self.get_logger().info(
            f'Goal {self._label} at ({self._target[0]:.2f}, {self._target[1]:.2f}), '
            f'boat at {twist.linear.x:.2f} m/s, {degrees(twist.angular.z):.1f} deg/s{aligned}')
        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = self._odom.header.frame_id
        goal.pose.position.x, goal.pose.position.y = map(float, self._target)
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self._set_state(GOAL, self._label)
        self._publish_markers()

    def _on_goal(self, now):
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
        self._clear_goal()
        if len(self._remaining):
            self._start_lead_in()
        else:
            self.get_logger().info(f'Done: {self._reached} reached, {self._missed} missed')
            self._set_state(DONE)

    def _set_state(self, state, status=None):
        """Set the state, and the status: the state's name unless given."""
        self._state = state
        self._set_status(status or state)

    def _set_status(self, status):
        if status != self._status:
            self._status = status
            self._status_pub.publish(String(data=status))

    # Lead-in

    def _fitting(self):
        """Place the remaining goals from the live pose; return the indices of those that fit."""
        xy = from_body(self._odom.pose.pose, self._remaining)
        self._area_xy = transform_xy(self._to_area, xy)
        self._publish_markers()
        return np.flatnonzero((np.abs(self._area_xy) <= self._half_size).all(axis=1))

    def _unready(self, fits, error):
        """Return why the boat isn't ready for a goal, or None; error is the heading error."""
        twist = self._odom.twist.twist
        if not len(fits):
            return 'no goal fits'
        if abs(twist.linear.x - self._start_speed) > self._speed_tolerance:
            return f'speed {twist.linear.x:.2f} m/s'
        if abs(twist.angular.z) > self._start_yaw_rate:
            return f'turning {degrees(twist.angular.z):.1f} deg/s'
        if self._state == RECENTRING and abs(error) > self._recentre_heading:
            return f'heading {degrees(error):.0f} deg'
        return None

    def _heading_error(self):
        """Return the angle from the bow to the heading to hold, or to the area's origin."""
        pose = self._odom.pose.pose
        if self._state == RECENTRING:
            return heading_error(pose, self._center)
        return wrap(self._heading - yaw_of(pose.orientation))

    def _drive(self, error):
        """Publish one step of the lead-in controller; the speed drops while the bow is off."""
        twist = self._odom.twist.twist
        speed = max(self._turn_speed, self._start_speed * cos(error))
        left, right = self._lead_in.update(
            error, speed, twist.linear.x, twist.angular.z, self._dt)
        self._drive_pub.publish(Drive(left=left, right=right))

    # Messages

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

    def _clear_goal(self):
        self._clear_pub.publish(Empty())
        self._current = None
        self._publish_markers()

    def _publish_markers(self):
        selected = self._area_xy[[self._current]] if self._current is not None else []
        self._markers_pub.publish(MarkerArray(markers=[
            marker(self._area_frame, 0, 0.3, ColorRGBA(g=0.8, b=0.8, a=0.8), self._area_xy),
            marker(self._area_frame, 1, 0.6, ColorRGBA(r=1.0, a=1.0), selected),
        ]))


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
