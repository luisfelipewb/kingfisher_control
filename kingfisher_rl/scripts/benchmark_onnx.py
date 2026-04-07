#!/usr/bin/env python3

import onnxruntime as ort
import numpy as np
import time
import argparse
import csv

def create_dummy_observations():
    """Create dummy observations matching the expected input format."""
    # Based on the original code, observations are concatenated as:
    # actions (2) + lin_vel (2) + ang_vel (1) + goal_cos_sin (2) + goal_distance (1) = 8 total
    
    actions = np.random.uniform(-1.0, 1.0, (1, 2)).astype(np.float32)
    lin_vel = np.random.uniform(-2.0, 2.0, (1, 2)).astype(np.float32)
    ang_vel = np.random.uniform(-1.0, 1.0, (1, 1)).astype(np.float32)
    goal_cos_sin = np.random.uniform(-1.0, 1.0, (1, 2)).astype(np.float32)
    goal_distance = np.random.uniform(0.0, 9.0, (1, 1)).astype(np.float32)
    
    observations = np.concatenate([
        actions,       # 2
        lin_vel,       # 2  
        ang_vel,       # 1
        goal_cos_sin,  # 2
        goal_distance  # 1
    ], axis=1)
    
    return {"obs": observations}

def benchmark_onnx_inference(model_path, num_iterations=1000):
    """Benchmark ONNX model inference."""
    
    print(f"Loading ONNX model from: {model_path}")
    
    # Load the ONNX model
    try:
        ort_session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    # Get model input/output info
    input_names = [inp.name for inp in ort_session.get_inputs()]
    output_names = [out.name for out in ort_session.get_outputs()]
    
    print(f"Model inputs: {input_names}")
    print(f"Model outputs: {output_names}")
    
    # Create dummy observations
    obs = create_dummy_observations()
    print(f"Input shape: {obs['obs'].shape}")
    
    # Warm-up runs
    print("Performing warm-up runs...")
    for _ in range(10):
        output = ort_session.run(None, obs)
    
    # Benchmark runs
    print(f"Running {num_iterations} inference iterations...")
    
    inference_times = []
    
    for i in range(num_iterations):
        # Create new random observations for each iteration
        obs = create_dummy_observations()
        
        start_time = time.time()
        output = ort_session.run(None, obs)
        end_time = time.time()
        
        inference_time = (end_time - start_time) * 1000  # Convert to milliseconds
        inference_times.append(inference_time)
        
        # Clip output as in original code
        action = np.clip(output[0].squeeze(0), -1, 1)
        
        if i % 100 == 0:
            print(f"Iteration {i}: {inference_time:.3f} ms, Action: [{action[0]:.3f}, {action[1]:.3f}]")
    

    csv_path = "/tmp/mlp_inference.csv"
    with open(csv_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["inference_time_ms"])
        for t in inference_times:
            writer.writerow([t])
    print(f"Inference times saved to {csv_path}")

    # Calculate statistics
    mean_time = np.mean(inference_times)
    std_time = np.std(inference_times)
    min_time = np.min(inference_times)
    max_time = np.max(inference_times)
    median_time = np.median(inference_times)
    
    print("\n" + "="*50)
    print("INFERENCE BENCHMARK RESULTS")
    print("="*50)
    print(f"Total iterations: {num_iterations}")
    print(f"Mean inference time: {mean_time:.3f} ± {std_time:.3f} ms")
    print(f"Median inference time: {median_time:.3f} ms")
    print(f"Min inference time: {min_time:.3f} ms")
    print(f"Max inference time: {max_time:.3f} ms")
    print(f"Total time: {sum(inference_times):.3f} ms")
    print(f"Throughput: {1000.0 / mean_time:.1f} inferences/second")
    
    # Calculate percentiles
    p95 = np.percentile(inference_times, 95)
    p99 = np.percentile(inference_times, 99)
    print(f"95th percentile: {p95:.3f} ms")
    print(f"99th percentile: {p99:.3f} ms")

def main():
    parser = argparse.ArgumentParser(description='Benchmark ONNX model inference')
    parser.add_argument('--model', '-m', type=str, required=True, 
                       help='Path to ONNX model file')
    parser.add_argument('--iterations', '-n', type=int, default=1000,
                       help='Number of inference iterations (default: 1000)')
    
    args = parser.parse_args()
    
    benchmark_onnx_inference(args.model, args.iterations)

if __name__ == '__main__':
    main()