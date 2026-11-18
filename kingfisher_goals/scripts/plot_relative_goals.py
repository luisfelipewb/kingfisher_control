#!/usr/bin/env python3
"""
Plots the trajectories of a relative_goal run from the CSV of relative_goal_csv.py.

Writes ``<name>_goals.png``, every goal's path in the frame of the boat when it was sent, with the
goals at their scenario ``[x, y]``, and ``<name>_run.png``, the whole run in odom's frame by phase.
"""

import argparse
import csv
from itertools import groupby
from math import cos, radians, sin
import os

import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')

PHASE_COLORS = {'goal': 'tab:blue', 'lead-in': 'tab:orange', 'recentring': 'tab:red'}


def column(rows, name):
    """Return a column as floats."""
    return [float(r[name]) for r in rows]


def plot_goals(rows, reach_distance, path):
    """Plot each goal's path in its start frame, x forward and y to port."""
    fig, ax = plt.subplots(figsize=(8, 8))
    goal_rows = [r for r in rows if r['phase'] == 'goal' and r['seq']]
    for _, group in groupby(goal_rows, key=lambda r: r['seq']):
        group = list(group)
        line, = ax.plot(column(group, 'sy'), column(group, 'sx'), linewidth=1)
        d, h = float(group[0]['goal_distance']), radians(float(group[0]['goal_heading_deg']))
        ax.add_patch(plt.Circle((d * sin(h), d * cos(h)), reach_distance, fill=False,
                                color=line.get_color()))
    ax.plot(0, 0, 'k^')
    ax.set_aspect('equal')
    ax.invert_xaxis()
    ax.set_xlabel('y, to port (m)')
    ax.set_ylabel('x, forward (m)')
    ax.set_title('Goals from the boat at the send')
    ax.grid(True)
    fig.savefig(path, dpi=150, bbox_inches='tight')


def plot_run(rows, path):
    """Plot the whole run in odom's frame, coloured by phase, with the goals."""
    fig, ax = plt.subplots(figsize=(8, 8))
    for phase, group in groupby(rows, key=lambda r: r['phase']):
        group = list(group)
        ax.plot(column(group, 'x'), column(group, 'y'), color=PHASE_COLORS.get(phase, 'grey'),
                linewidth=1)
    for _, group in groupby([r for r in rows if r['seq']], key=lambda r: r['seq']):
        r = next(group)
        ax.plot(float(r['goal_x']), float(r['goal_y']), 'kx')
        ax.annotate(r['goal_index'], (float(r['goal_x']), float(r['goal_y'])), fontsize=7)
    for phase, color in PHASE_COLORS.items():
        ax.plot([], [], color=color, label=phase)
    ax.legend()
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Run in odom')
    ax.grid(True)
    fig.savefig(path, dpi=150, bbox_inches='tight')


def main():
    """Write the two plots next to the CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='samples CSV from relative_goal_csv.py')
    parser.add_argument('--reach-distance', type=float, default=0.33, help='m, the goal circles')
    args = parser.parse_args()

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    name = os.path.splitext(args.csv)[0]
    plot_goals(rows, args.reach_distance, f'{name}_goals.png')
    plot_run(rows, f'{name}_run.png')
    print(f'{name}_goals.png\n{name}_run.png')


if __name__ == '__main__':
    main()
