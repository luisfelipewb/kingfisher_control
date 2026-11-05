#!/usr/bin/env python3
"""
Writes a relative_goal scenario: a grid of distances and bearings, shuffled.

Run it by hand from the package's source tree; the file goes in ``config/scenarios/``. A positive
bearing is to port.
"""

import argparse
import sys

import numpy as np


def steps(low, high, step):
    """Return the values from low to high, both included."""
    return np.arange(low, high + step / 2.0, step)


def main():
    """Write the scenario."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', nargs=3, type=float, required=True,
                        metavar=('MIN', 'MAX', 'STEP'), help='m')
    parser.add_argument('--bearing', nargs=3, type=float, required=True,
                        metavar=('MIN', 'MAX', 'STEP'), help='deg')
    parser.add_argument('--repeat', type=int, default=1, help='times each goal is in the list')
    parser.add_argument('--seed', type=int, default=0, help='of the shuffle')
    parser.add_argument('--output', required=True, help='scenario YAML')
    args = parser.parse_args()

    distance, bearing = np.meshgrid(steps(*args.distance), np.radians(steps(*args.bearing)))
    goals = np.column_stack((
        (distance * np.cos(bearing)).ravel(), (distance * np.sin(bearing)).ravel()))
    goals = np.random.default_rng(args.seed).permutation(np.tile(goals, (args.repeat, 1)))

    with open(args.output, 'w') as f:
        f.write(f'# scripts/generate_goals.py {" ".join(sys.argv[1:])}\n')
        f.write('goals:                 # robot frame, m: x forward, y to port\n')
        for x, y in goals:
            f.write(f'  - [{x:.3f}, {y:.3f}]\n')


if __name__ == '__main__':
    main()
