#!/usr/bin/env python3
"""
Turns a relative_goal bag into CSV files: one row per action, and one per goal.

Run it by hand with ROS sourced, on a bag from relative_goal.launch.py. ``<bag>.csv`` has a row
per ``control/cmd_drive`` message, the policy's actions and the lead-in's, with odom interpolated
to it; ``sx``, ``sy``, ``syaw`` are the pose in the frame of the boat when the goal was sent, where
the goal is at its scenario ``[x, y]``. ``<bag>_goals.csv`` has a row per goal sent; ``seq`` joins
the two.
Times are the bag's, in s from its first message; angles in rad unless named ``_deg``.
"""

import argparse
import csv
from math import atan2, cos, degrees, hypot, sin
import os

from kingfisher_goals.geometry import wrap, yaw_of
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

SAMPLE_COLUMNS = [
    't', 'phase', 'seq', 'goal_index', 'goal_distance', 'goal_heading_deg', 'left', 'right',
    'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'goal_x', 'goal_y', 'goal_range', 'goal_bearing_deg',
    'sx', 'sy', 'syaw']
GOAL_COLUMNS = [
    'seq', 'goal_index', 'goal_distance', 'goal_heading_deg', 't_start', 't_end', 'duration',
    'outcome', 'closest', 'final_range', 'start_speed', 'start_yaw_rate']


def read(bag):
    """Return the bag's odom, actions, status and goals, each as a list of (t, message)."""
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    namespace = next(name for name in types if name.endswith('/odom'))[:-len('odom')]
    topics = {namespace + name: name for name in
              ('odom', 'control/cmd_drive', 'relative_goal/status', 'goal')}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    data = {name: [] for name in topics.values()}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        data[topics[topic]].append((stamp * 1e-9, msg))
    return data


class Odom:
    """Odom's pose and body twist, interpolated at any time."""

    def __init__(self, samples):
        self.t = np.array([t for t, _ in samples])
        fields = [(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
                   m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z)
                  for _, m in samples]
        self.values = np.array(fields).T
        self.values[2] = np.unwrap(self.values[2])

    def at(self, t):
        """Return x, y, yaw, vx, vy, wz at t (a scalar or an array)."""
        x, y, yaw, vx, vy, wz = (np.interp(t, self.t, v) for v in self.values)
        return x, y, np.arctan2(np.sin(yaw), np.cos(yaw)), vx, vy, wz


def goal_segments(status, goals, odom, end, reach_distance):
    """Return a dict per goal sent: its fields, its time span, start pose and outcome."""
    segments = []
    for i, (t, msg) in enumerate(status):
        fields = msg.data.split()
        if fields[0] != 'goal':
            continue
        nxt = status[i + 1] if i + 1 < len(status) else None
        t_end = nxt[0] if nxt else end
        goal = goals[len(segments)][1].pose.position
        sx, sy, syaw, speed, _, yaw_rate = odom.at(t)
        span = (odom.t >= t) & (odom.t <= t_end)
        ranges = np.hypot(goal.x - odom.values[0][span], goal.y - odom.values[1][span])
        ex, ey = odom.at(t_end)[:2]
        final = hypot(goal.x - ex, goal.y - ey)
        if nxt is None:
            outcome = 'unfinished'
        elif nxt[1].data.split()[0] == 'waiting':
            outcome = 'put back'
        else:
            outcome = 'reached' if final < reach_distance else 'missed'
        segments.append({
            'seq': len(segments) + 1, 'goal_index': int(fields[1]),
            'goal_distance': float(fields[2]), 'goal_heading_deg': float(fields[3]),
            't_start': t, 't_end': t_end, 'duration': t_end - t, 'outcome': outcome,
            'closest': min(final, *ranges), 'final_range': final,
            'start_speed': speed, 'start_yaw_rate': yaw_rate,
            'goal_x': goal.x, 'goal_y': goal.y, 'start': (sx, sy, syaw)})
    return segments


def sample_rows(data, odom, segments, t0):
    """Yield a row per action inside odom's time span."""
    status_t = np.array([t for t, _ in data['relative_goal/status']])
    starts = np.array([s['t_start'] for s in segments])
    for t, msg in data['control/cmd_drive']:
        if not odom.t[0] <= t <= odom.t[-1]:
            continue
        x, y, yaw, vx, vy, wz = odom.at(t)
        i = np.searchsorted(status_t, t, 'right') - 1
        phase = data['relative_goal/status'][i][1].data.split()[0] if i >= 0 else ''
        row = {'t': t - t0, 'phase': phase, 'left': msg.left, 'right': msg.right,
               'x': x, 'y': y, 'yaw': yaw, 'vx': vx, 'vy': vy, 'wz': wz}
        j = np.searchsorted(starts, t, 'right') - 1
        if phase == 'goal' and j >= 0:
            s = segments[j]
            gx, gy = s['goal_x'], s['goal_y']
            px, py, pyaw = s['start']
            c, si = cos(pyaw), sin(pyaw)
            row.update({key: s[key] for key in ('seq', 'goal_index', 'goal_distance',
                                                'goal_heading_deg')})
            row.update({
                'goal_x': gx, 'goal_y': gy, 'goal_range': hypot(gx - x, gy - y),
                'goal_bearing_deg': degrees(wrap(atan2(gy - y, gx - x) - yaw)),
                'sx': c * (x - px) + si * (y - py), 'sy': -si * (x - px) + c * (y - py),
                'syaw': wrap(yaw - pyaw)})
        yield row


def write(path, columns, rows):
    """Write rows of dicts to a CSV, floats with 4 decimals, missing values empty."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        for row in rows:
            writer.writerow([f'{row[c]:.4f}' if isinstance(row.get(c), (float, np.floating))
                             else row.get(c, '') for c in columns])


def main():
    """Write the two CSVs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='bag folder')
    parser.add_argument('--output', help='folder of the CSVs; next to the bag by default')
    parser.add_argument('--reach-distance', type=float, default=0.33,
                        help='m, as relative_goal: a goal closer than this at its end is reached')
    args = parser.parse_args()

    bag = os.path.normpath(args.bag)
    output = args.output or os.path.dirname(os.path.abspath(bag))
    name = os.path.join(output, os.path.basename(bag))
    data = read(bag)
    odom = Odom(data['odom'])
    t0 = min(samples[0][0] for samples in data.values() if samples)
    segments = goal_segments(data['relative_goal/status'], data['goal'], odom, odom.t[-1],
                             args.reach_distance)

    rows = list(sample_rows(data, odom, segments, t0))
    write(f'{name}.csv', SAMPLE_COLUMNS, rows)
    for s in segments:
        s['t_start'] -= t0
        s['t_end'] -= t0
    write(f'{name}_goals.csv', GOAL_COLUMNS, segments)
    outcomes = {o: sum(s['outcome'] == o for s in segments) for o in
                ('reached', 'missed', 'put back', 'unfinished')}
    print(f'{name}.csv: {len(rows)} actions\n{name}_goals.csv: {len(segments)} goals, '
          + ', '.join(f'{n} {o}' for o, n in outcomes.items() if n))


if __name__ == '__main__':
    main()
