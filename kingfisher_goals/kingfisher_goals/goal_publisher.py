"""
Turns a goal in any frame into a body-frame goal stream for the controllers.

* a ``goal`` (geometry_msgs/PoseStamped) is transformed once into ``odom``'s frame (tf2,
  latest) and replaces the previous one;
* at ``rate``, the goal is published on ``control/goal`` in ``odom``'s child frame
  (``kingfisher/base_link``), stamped with the ``odom`` stamp, in 2D: z, roll and pitch are 0;
* the goal is cleared once closer than ``reach_distance``, or ``goal_timeout`` after it was
  received (receipt time, not its stamp, so the sender's clock doesn't matter);
* nothing goes on ``control/goal`` while ``odom`` is older than ``odom_timeout``;
* ``control/status`` (std_msgs/Bool) is true while a goal is active, at ``rate``.
"""

from math import atan2, cos, hypot, sin

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    """Return the yaw of a geometry_msgs/Quaternion."""
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalPublisher(Node):
    """Publish the active goal in the body frame at a fixed rate."""

    def __init__(self):
        super().__init__('goal_publisher')

        rate = self.declare_parameter('rate', 10.0).value
        # m, the goal is reached closer than this
        self._reach_distance = self.declare_parameter('reach_distance', 0.1).value
        # s after a goal is received before it is given up
        self._goal_timeout = Duration(
            seconds=self.declare_parameter('goal_timeout', 30.0).value)
        # s, older odom stops the goal stream
        self._odom_timeout = Duration(
            seconds=self.declare_parameter('odom_timeout', 1.0).value)

        self._odom = None
        # (x, y, yaw) in odom's frame, and its receipt time
        self._goal = None
        self._goal_time = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal_pub = self.create_publisher(PoseStamped, 'control/goal', 1)
        self._status_pub = self.create_publisher(Bool, 'control/status', 1)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_subscription(PoseStamped, 'goal', self._goal_callback, 10)
        self.create_timer(1.0 / rate, self._update)

    def _odom_callback(self, msg):
        self._odom = msg

    def _goal_callback(self, msg):
        if self._odom is None:
            self.get_logger().warn('Goal ignored: no odom yet')
            return
        frame = self._odom.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(frame, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'Goal ignored: {e}')
            return
        pose = do_transform_pose_stamped(msg, transform).pose
        if self._goal is None:
            self.get_logger().info(
                f'New goal ({pose.position.x:.2f}, {pose.position.y:.2f}) in {frame}')
        self._goal = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self._goal_time = self.get_clock().now()

    def _update(self):
        now = self.get_clock().now()
        if self._goal is not None and now - self._goal_time > self._goal_timeout:
            self.get_logger().info('Goal timed out')
            self._goal = None
        msg = self._body_goal(now) if self._goal is not None else None
        self._status_pub.publish(Bool(data=self._goal is not None))
        if msg is not None:
            self._goal_pub.publish(msg)

    def _body_goal(self, now):
        """Return the goal in odom's child frame, or None (and clear it once reached)."""
        odom = self._odom
        if now - Time.from_msg(odom.header.stamp) > self._odom_timeout:
            self.get_logger().warn('Stale odom: goal not published', throttle_duration_sec=1.0)
            return None

        p = odom.pose.pose.position
        yaw = yaw_of(odom.pose.pose.orientation)
        gx, gy, gyaw = self._goal
        dx, dy = gx - p.x, gy - p.y
        x = cos(yaw) * dx + sin(yaw) * dy
        y = -sin(yaw) * dx + cos(yaw) * dy
        if hypot(x, y) < self._reach_distance:
            self.get_logger().info('Goal reached')
            self._goal = None
            return None

        msg = PoseStamped()
        msg.header.stamp = odom.header.stamp
        msg.header.frame_id = odom.child_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = sin((gyaw - yaw) / 2.0)
        msg.pose.orientation.w = cos((gyaw - yaw) / 2.0)
        return msg


def main(args=None):
    """Spin the goal publisher."""
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
