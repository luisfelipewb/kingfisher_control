#!/usr/bin/env python3
"""
Time an ONNX policy's inference on random passthrough_thrusters observations.

    python3 scripts/benchmark_onnx.py -m models/passthrough_thrusters/seed44.onnx
"""

import argparse
import csv
import time

import numpy as np
import onnxruntime as ort


def random_observation():
    """Return a random observation: action, vx vy, wz, cos sin of the bearing, distance."""
    return np.concatenate([
        np.random.uniform(-1.0, 1.0, 2),
        np.random.uniform(-2.0, 2.0, 2),
        np.random.uniform(-1.0, 1.0, 1),
        np.random.uniform(-1.0, 1.0, 2),
        np.random.uniform(0.0, 9.0, 1),
    ]).astype(np.float32).reshape(1, -1)


def benchmark(model, iterations, csv_path):
    """Run the policy `iterations` times and print the timing statistics."""
    session = ort.InferenceSession(model, providers=['CPUExecutionProvider'])
    name = session.get_inputs()[0].name
    print(f'Inputs: {[i.name for i in session.get_inputs()]}, '
          f'outputs: {[o.name for o in session.get_outputs()]}')

    for _ in range(10):
        session.run(None, {name: random_observation()})

    times = []
    for i in range(iterations):
        obs = random_observation()
        start = time.perf_counter()
        output = session.run(None, {name: obs})
        times.append((time.perf_counter() - start) * 1000.0)
        if i % 100 == 0:
            action = np.clip(output[0].reshape(-1), -1.0, 1.0)
            print(f'{i}: {times[-1]:.3f} ms, action {np.round(action, 3)}')

    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['inference_time_ms'])
            writer.writerows([t] for t in times)
        print(f'Times saved to {csv_path}')

    times = np.array(times)
    print(f'{iterations} runs: mean {times.mean():.3f} ± {times.std():.3f} ms, '
          f'median {np.median(times):.3f}, min {times.min():.3f}, max {times.max():.3f}, '
          f'p95 {np.percentile(times, 95):.3f}, p99 {np.percentile(times, 99):.3f} ms; '
          f'{1000.0 / times.mean():.0f} inferences/s')


def main():
    """Parse the arguments and run the benchmark."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--model', '-m', required=True, help='ONNX policy')
    parser.add_argument('--iterations', '-n', type=int, default=1000)
    parser.add_argument('--csv', help='save the times to this CSV file')
    args = parser.parse_args()
    benchmark(args.model, args.iterations, args.csv)


if __name__ == '__main__':
    main()
