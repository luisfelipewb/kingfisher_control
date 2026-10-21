"""
Passthrough with the thrusters: drive through the goal position.

The observation is the one the policy was trained on, from the ROS 1 ``capture_agent_onnx.py``:
the previous action (2), the ``odom`` twist vx, vy (2) and wz (1) in the body frame, the cos and
sin of the goal bearing (2), and the goal distance capped at 9 m (1). The action is
``control/cmd_drive`` (kingfisher_msgs/Drive) left and right.
"""

from math import atan2, cos, hypot, sin

from kingfisher_msgs.msg import Drive
from kingfisher_rl.agent_base import Agent
import rclpy
from rclpy.executors import ExternalShutdownException

# m, the goal distance the policy sees at most
MAX_DISTANCE = 9.0


class PassthroughThrusters(Agent):
    """Drive through the goal with the thrusters."""

    def __init__(self):
        super().__init__('passthrough_thrusters')
        self._cmd_pub = self.create_publisher(Drive, 'control/cmd_drive', 1)

    def observe(self, goal):
        """Return the 8 observations for a body-frame goal."""
        twist = self.odom.twist.twist
        x, y = goal.pose.position.x, goal.pose.position.y
        bearing = atan2(y, x)
        return [
            *self.action,
            twist.linear.x, twist.linear.y,
            twist.angular.z,
            cos(bearing), sin(bearing),
            min(hypot(x, y), MAX_DISTANCE),
        ]

    def act(self, action):
        """Publish the action as left and right thrust."""
        self._cmd_pub.publish(Drive(left=float(action[0]), right=float(action[1])))


def main(args=None):
    """Spin the passthrough agent."""
    rclpy.init(args=args)
    node = PassthroughThrusters()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
