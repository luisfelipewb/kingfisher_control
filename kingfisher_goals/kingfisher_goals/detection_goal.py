"""
Picks a goal from detections.

On each ``detections`` (sensor_msgs/PointCloud2, any frame), the closest point within
``max_goal_distance`` of the frame's origin goes on ``goal`` (geometry_msgs/PoseStamped) with the
cloud's header. Nothing is sent when no point is close enough.
"""

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class DetectionGoal(Node):
    """Publish the closest detection as a goal."""

    def __init__(self):
        super().__init__('detection_goal')

        # m, farther detections are ignored
        self._max_goal_distance = self.declare_parameter('max_goal_distance', 5.0).value

        self._goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self.create_subscription(PointCloud2, 'detections', self._detections_callback, 1)

    def _detections_callback(self, msg):
        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(points) == 0:
            return
        distances = np.linalg.norm(points, axis=1)
        i = int(np.argmin(distances))
        if distances[i] > self._max_goal_distance:
            self.get_logger().warn(
                f'Closest detection too far: {distances[i]:.1f} m', throttle_duration_sec=1.0)
            return

        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = (
            float(v) for v in points[i])
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)


def main(args=None):
    """Spin the detection goal node."""
    rclpy.init(args=args)
    node = DetectionGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
