"""
The shared part of the RL agents: an ONNX policy run once per goal.

On each ``control/goal`` (geometry_msgs/PoseStamped in ``odom``'s child frame, from
``kingfisher_goals``' ``goal_publisher``), with fresh ``odom``: ``observe()`` → inference →
clip to [-1, 1] → ``act()``. There is no timer: when goals stop, commands stop. The previous
action, part of most observations, resets to 0 after ``episode_gap`` without goals.

``policy`` is a path relative to ``share/kingfisher_rl/models/``, or an absolute path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import onnxruntime as ort
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


class Agent(Node):
    """Run an ONNX policy on each goal; subclasses define observe() and act()."""

    # Length of the action vector
    action_size = 2

    def __init__(self, name):
        super().__init__(name)

        policy = self.declare_parameter('policy', '').value
        # s, older odom skips the goal
        self._odom_timeout = Duration(
            seconds=self.declare_parameter('odom_timeout', 1.0).value)
        # s without goals before the previous action resets (a new episode)
        self._episode_gap = Duration(
            seconds=self.declare_parameter('episode_gap', 0.5).value)

        path = os.path.join(get_package_share_directory('kingfisher_rl'), 'models', policy)
        self._session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
        self._input = self._session.get_inputs()[0].name
        self.get_logger().info(f'Policy {path}')

        self.odom = None
        self.action = np.zeros(self.action_size, dtype=np.float32)
        self._last_goal_time = None

        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_subscription(PoseStamped, 'control/goal', self._goal_callback, 1)

    def observe(self, goal):
        """Return the observation vector for a goal in the body frame."""
        raise NotImplementedError

    def act(self, action):
        """Send an action, clipped to [-1, 1]."""
        raise NotImplementedError

    def _odom_callback(self, msg):
        self.odom = msg

    def _goal_callback(self, msg):
        now = self.get_clock().now()
        if self.odom is None:
            self.get_logger().warn('Goal skipped: no odom yet', throttle_duration_sec=1.0)
            return
        if msg.header.frame_id != self.odom.child_frame_id:
            self.get_logger().warn(
                f'Goal skipped: in {msg.header.frame_id}, not {self.odom.child_frame_id}',
                throttle_duration_sec=1.0)
            return
        if now - Time.from_msg(self.odom.header.stamp) > self._odom_timeout:
            self.get_logger().warn('Goal skipped: stale odom', throttle_duration_sec=1.0)
            return

        if self._last_goal_time is None or now - self._last_goal_time > self._episode_gap:
            self.get_logger().info('New episode')
            self.action[:] = 0.0
        self._last_goal_time = now

        obs = np.asarray(self.observe(msg), dtype=np.float32).reshape(1, -1)
        output = self._session.run(None, {self._input: obs})[0]
        self.action = np.clip(output.reshape(-1), -1.0, 1.0).astype(np.float32)
        self.act(self.action)
