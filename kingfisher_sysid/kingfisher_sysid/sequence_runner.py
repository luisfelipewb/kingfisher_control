"""
Plays an open-loop thrust sequence and logs the boat's response to CSV.

``experiment`` is an ``experiment_name`` in ``experiments_file`` (default
``config/experiments.yaml``): a list of sections, each a thrust pair ``tl``, ``tr`` in [-1, 1]
held for ``duration`` s. Once ``odom`` arrives, each section is sent on ``control/cmd_drive`` at
``rate``, with ``odom`` logged on every command. After the last section the CSV goes to
``output_dir``/<experiment>.csv and the node exits. ``odom``'s twist is already in the body
frame, so it is logged as it comes.
"""

import csv
from math import asin, atan2
import os

from ament_index_python.packages import get_package_share_directory
from kingfisher_msgs.msg import Drive
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml

COLUMNS = ['time', 'thr_r', 'thr_l', 'pos_x', 'pos_y', 'lin_x', 'lin_y', 'ang_z',
           'roll', 'pitch', 'yaw']


def euler_of(q):
    """Return roll, pitch, yaw of a geometry_msgs/Quaternion."""
    roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class SequenceRunner(Node):
    """Send a thrust sequence and record odom."""

    def __init__(self):
        super().__init__('sequence_runner')

        self._experiment = self.declare_parameter('experiment', 'acceleration_1.0').value
        experiments_file = self.declare_parameter(
            'experiments_file',
            os.path.join(get_package_share_directory('kingfisher_sysid'),
                         'config', 'experiments.yaml')).value
        self._output_dir = os.path.expanduser(self.declare_parameter('output_dir', '.').value)
        rate = self.declare_parameter('rate', 10.0).value

        with open(experiments_file) as f:
            experiments = {e['experiment_name']: e['sections'] for e in yaml.safe_load(f)}
        if self._experiment not in experiments:
            raise KeyError(f'No experiment {self._experiment} in {experiments_file}')
        self._sections = experiments[self._experiment]

        self._odom = None
        self._start = None
        self._section = 0
        self._section_start = None
        self._rows = []
        self.done = False

        self._cmd_pub = self.create_publisher(Drive, 'control/cmd_drive', 1)
        self._name_pub = self.create_publisher(String, 'experiment_name', 1)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.create_timer(1.0 / rate, self._update)

    def _odom_callback(self, msg):
        self._odom = msg

    def _update(self):
        if self.done:
            return
        if self._odom is None:
            self.get_logger().info('Waiting for odom', throttle_duration_sec=1.0)
            return
        now = self.get_clock().now()
        if self._start is None:
            self._start = self._section_start = now
            self._log_section()

        while (now - self._section_start).nanoseconds * 1e-9 >= \
                self._sections[self._section]['duration']:
            self._section += 1
            if self._section == len(self._sections):
                self._save()
                self.done = True
                return
            self._section_start = now
            self._log_section()

        section = self._sections[self._section]
        cmd = Drive(left=float(section['tl']), right=float(section['tr']))
        self._cmd_pub.publish(cmd)
        self._name_pub.publish(String(data=self._experiment))
        self._record((now - self._start).nanoseconds * 1e-9, cmd)

    def _log_section(self):
        s = self._sections[self._section]
        self.get_logger().info(
            f'{self._experiment} {self._section + 1}/{len(self._sections)}: '
            f'tl={s["tl"]}, tr={s["tr"]} for {s["duration"]} s')

    def _record(self, t, cmd):
        pose, twist = self._odom.pose.pose, self._odom.twist.twist
        roll, pitch, yaw = euler_of(pose.orientation)
        self._rows.append([
            f'{t:.3f}', cmd.right, cmd.left,
            f'{pose.position.x:.4f}', f'{pose.position.y:.4f}',
            f'{twist.linear.x:.4f}', f'{twist.linear.y:.4f}', f'{twist.angular.z:.4f}',
            f'{roll:.4f}', f'{pitch:.4f}', f'{yaw:.4f}'])

    def _save(self):
        os.makedirs(self._output_dir, exist_ok=True)
        path = os.path.join(self._output_dir, f'{self._experiment}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows(self._rows)
        self.get_logger().info(f'Done: {len(self._rows)} rows in {os.path.abspath(path)}')


def main(args=None):
    """Run one experiment, then exit."""
    rclpy.init(args=args)
    node = SequenceRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
