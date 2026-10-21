"""
Virtual bottles, seen as a detector would see them; in sim or on the water.

The bottles come from ``bottles_file`` (a scenario YAML: ``frame_id`` and
``bottles: [[x, y], ...]``) and are transformed once into ``odom``'s frame. A bottle is
collected, and gone for good, once the boat is closer than ``collect_radius``. At ``rate``, the
bottles within ``max_range`` and ``half_fov_deg`` of the bow go on ``detections``
(sensor_msgs/PointCloud2 in ``odom``'s child frame, with the ``odom`` stamp), and all remaining
ones on ``virtual_detections/bottles`` (in ``odom``'s frame), for display.
"""

from kingfisher_goals.geometry import to_body, transform_xy
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def cloud(header, xy):
    """Return a PointCloud2 of (N, 2) points at z = 0."""
    return point_cloud2.create_cloud_xyz32(header, [(x, y, 0.0) for x, y in xy])


class VirtualDetections(Node):
    """Publish detections of virtual bottles, and collect them."""

    def __init__(self):
        super().__init__('virtual_detections')

        bottles_file = self.declare_parameter('bottles_file', '').value
        rate = self.declare_parameter('rate', 2.0).value
        # m, a bottle is collected closer than this
        self._collect_radius = self.declare_parameter('collect_radius', 0.33).value
        # m and deg either side of the bow, what the detector sees
        self._max_range = self.declare_parameter('max_range', 10.0).value
        self._half_fov = np.radians(self.declare_parameter('half_fov_deg', 35.0).value)

        with open(bottles_file) as f:
            scenario = yaml.safe_load(f)
        self._frame = scenario['frame_id']
        self._raw = np.array(scenario.get('bottles') or [], dtype=float).reshape(-1, 2)
        self.get_logger().info(f'{len(self._raw)} bottles from {bottles_file}')

        self._odom = None
        # (N, 2) in odom's frame, once transformed
        self._bottles = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._detections_pub = self.create_publisher(PointCloud2, 'detections', 1)
        self._bottles_pub = self.create_publisher(PointCloud2, 'virtual_detections/bottles', 1)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_timer(1.0 / rate, self._update)

    def _odom_callback(self, msg):
        self._odom = msg
        if self._bottles is None or len(self._bottles) == 0:
            return
        p = msg.pose.pose.position
        collected = np.hypot(*(self._bottles - [p.x, p.y]).T) < self._collect_radius
        if collected.any():
            self._bottles = self._bottles[~collected]
            self.get_logger().info(f'Bottle collected, {len(self._bottles)} left')

    def _update(self):
        if self._odom is None:
            return
        if self._bottles is None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._odom.header.frame_id, self._frame, Time())
            except TransformException as e:
                self.get_logger().warn(f'No transform: {e}', throttle_duration_sec=1.0)
                return
            self._bottles = transform_xy(transform, self._raw)

        body = to_body(self._odom.pose.pose, self._bottles)
        seen = (np.hypot(*body.T) < self._max_range) & \
            (np.abs(np.arctan2(body[:, 1], body[:, 0])) < self._half_fov)
        self._detections_pub.publish(cloud(
            Header(stamp=self._odom.header.stamp, frame_id=self._odom.child_frame_id),
            body[seen]))
        self._bottles_pub.publish(cloud(
            Header(stamp=self._odom.header.stamp, frame_id=self._odom.header.frame_id),
            self._bottles))


def main(args=None):
    """Spin the virtual detections node."""
    rclpy.init(args=args)
    node = VirtualDetections()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
