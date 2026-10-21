"""
Follows a list of waypoints, and diverts to detections near the current one.

* the list comes on ``waypoints`` (geometry_msgs/PoseArray, any frame) or from
  ``waypoints_file`` (a scenario YAML: ``frame_id`` and ``waypoints: [[x, y], ...]``). A new list
  replaces the current one and starts over; it is transformed once into ``odom``'s frame and
  republished on ``waypoints/active``;
* a waypoint is done once the boat is closer than ``reach_distance``; after the last one, nothing
  more is sent (or the list starts over with ``loop``);
* at ``rate``, the current target goes on ``goal`` in ``odom``'s frame: the detection
  (``detections``, sensor_msgs/PointCloud2, any frame) closest to the boat within
  ``capture_radius`` of the current waypoint, while one came in the last ``detection_timeout``;
  otherwise the waypoint.
"""

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from kingfisher_goals.geometry import transform_xy
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class WaypointGoal(Node):
    """Send the current waypoint, or a detection near it, as the goal."""

    def __init__(self):
        super().__init__('waypoint_goal')

        waypoints_file = self.declare_parameter('waypoints_file', '').value
        rate = self.declare_parameter('rate', 2.0).value
        # m, a waypoint is done closer than this
        self._reach_distance = self.declare_parameter('reach_distance', 2.0).value
        # m from the current waypoint within which a detection is taken
        self._capture_radius = self.declare_parameter('capture_radius', 7.0).value
        # s without a usable detection before going back to the waypoint
        self._detection_timeout = Duration(
            seconds=self.declare_parameter('detection_timeout', 1.0).value)
        self._loop = self.declare_parameter('loop', False).value

        self._odom = None
        # (frame_id, (N, 2) points) not yet in odom's frame
        self._pending = None
        # (N, 2) in odom's frame, and the index of the current one (None when done)
        self._waypoints = None
        self._index = None
        # (x, y) in odom's frame, and its receipt time
        self._detection = None
        self._detection_time = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self._active_pub = self.create_publisher(PoseArray, 'waypoints/active', latched)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_subscription(PoseArray, 'waypoints', self._waypoints_callback, 1)
        self.create_subscription(PointCloud2, 'detections', self._detections_callback, 1)
        self.create_timer(1.0 / rate, self._update)

        if waypoints_file:
            with open(waypoints_file) as f:
                scenario = yaml.safe_load(f)
            self._pending = (scenario['frame_id'], np.array(scenario['waypoints'], dtype=float))
            self.get_logger().info(
                f'{len(self._pending[1])} waypoints from {waypoints_file}')

    def _waypoints_callback(self, msg):
        xy = np.array([[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        self._pending = (msg.header.frame_id, xy)
        self.get_logger().info(f'{len(xy)} waypoints in {msg.header.frame_id}')

    def _odom_callback(self, msg):
        self._odom = msg
        if self._index is None:
            return
        p = msg.pose.pose.position
        if np.hypot(*(self._waypoints[self._index] - [p.x, p.y])) >= self._reach_distance:
            return
        self._index += 1
        self._detection = None
        if self._index == len(self._waypoints):
            self._index = 0 if self._loop else None
        if self._index is None:
            self.get_logger().info('Last waypoint reached')
        else:
            self.get_logger().info(f'Waypoint {self._index + 1}/{len(self._waypoints)}')

    def _detections_callback(self, msg):
        if self._index is None:
            return
        xy = self._to_odom(msg.header.frame_id, point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y'), skip_nans=True))
        if xy is None or len(xy) == 0:
            return
        near = xy[np.hypot(*(xy - self._waypoints[self._index]).T) < self._capture_radius]
        if len(near) == 0:
            return
        p = self._odom.pose.pose.position
        self._detection = near[np.argmin(np.hypot(*(near - [p.x, p.y]).T))]
        self._detection_time = self.get_clock().now()

    def _update(self):
        if self._pending is not None and self._odom is not None:
            xy = self._to_odom(*self._pending)
            if xy is not None:
                self._activate(xy)
        if self._index is None:
            return

        now = self.get_clock().now()
        target = self._waypoints[self._index]
        if self._detection is not None and \
                now - self._detection_time < self._detection_timeout:
            target = self._detection

        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = self._odom.header.frame_id
        goal.pose.position.x, goal.pose.position.y = float(target[0]), float(target[1])
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

    def _activate(self, xy):
        self._pending = None
        self._waypoints = xy
        self._index = 0 if len(xy) else None
        self._detection = None
        active = PoseArray()
        active.header.stamp = self.get_clock().now().to_msg()
        active.header.frame_id = self._odom.header.frame_id
        for x, y in xy:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = 1.0
            active.poses.append(pose)
        self._active_pub.publish(active)
        self.get_logger().info(f'Following {len(xy)} waypoints')

    def _to_odom(self, frame, xy):
        """Return (N, 2) points from frame in odom's frame, or None without TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom.header.frame_id, frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No transform: {e}', throttle_duration_sec=1.0)
            return None
        return transform_xy(transform, xy)


def main(args=None):
    """Spin the waypoint goal node."""
    rclpy.init(args=args)
    node = WaypointGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
