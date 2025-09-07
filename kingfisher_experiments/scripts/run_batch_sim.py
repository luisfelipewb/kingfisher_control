#!/usr/bin/env python3

import os
import time
import subprocess
import signal
import yaml

import roslaunch
import rospkg
import argparse


def load_experiments(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config["experiments"]

class ProcessManager:
    def __init__(self):
        self.processes = {}

    def start(self, name, cmd, sleep_time=0):
        proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        self.processes[name] = proc
        if sleep_time > 0:
            time.sleep(sleep_time)
        print(f"{name} started.")
        return proc

    def stop(self, name):
        proc = self.processes.get(name)
        if proc:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait()
            print(f"{name} terminated.")
            del self.processes[name]

class ROSLaunchManager:
    def __init__(self):
        self.launches = {}

    def start(self, name, package, launch_file, args=None, sleep_time=0):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_path = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0]
        launch_args = []
        if args:
            for k, v in args.items():
                launch_args.append(f"{k}:={v}")
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_path, launch_args)])
        launch.start()
        self.launches[name] = launch
        if sleep_time > 0:
            time.sleep(sleep_time)
        print(f"{name} ROS launch started.")
        return launch

    def stop(self, name):
        launch = self.launches.get(name)
        if launch:
            launch.shutdown()
            print(f"{name} ROS launch terminated.")
            del self.launches[name]

def rosservice_call(service, args=""):
    print(f"Calling ROS service: {service} with args: {args}")
    subprocess.call(f"rosservice call {service} {args}", shell=True)

def main():

    # Load experiments from argument and ros package directory
    rospack = rospkg.RosPack()
    parser = argparse.ArgumentParser(description="Run batch simulation experiments.")
    parser.add_argument("--config", type=str, default="simulation_batch.yaml", help="YAML config file name")
    args = parser.parse_args()
    config_file_name = args.config
    config_file_path = rospack.get_path('kingfisher_experiments') + f'/config/{config_file_name}'
    experiments = load_experiments(config_file_path)

    # Set default values for parameters
    defaults = {
        "base_dir": "/tmp/",
        "exp_name": "default_exp_name",
        "onnx_path": "",
        "localization_delay": 0.0,
        "loe_right": 1.0,
        "pixel_noise_radius": 0,
        "mass": 35.0,
        "cog_y": 0.0,
        "sdq": 17.26,
    }

    for params in experiments:

        # Load parameters, use default if not present
        base_dir = params.get("base_dir", defaults["base_dir"])
        exp_name = params.get("exp_name", defaults["exp_name"])
        onnx_path = params.get("onnx_path", defaults["onnx_path"])
        localization_delay = params.get("localization_delay", defaults["localization_delay"])
        loe_right = params.get("loe_right", defaults["loe_right"])
        pixel_noise_radius = params.get("pixel_noise_radius", defaults["pixel_noise_radius"])
        mass = params.get("mass", defaults["mass"])
        cog_y = params.get("cog_y", defaults["cog_y"])
        sdq = params.get("sdq", defaults["sdq"])

        # Hardcoded parameters
        exp_file = "experiments.yaml"
        dst_thres = 0.3
        mi_z = "8.061"
        rdl = "5.83"
        rdq = "17.34"
        sdl = "0.0"

        pm = ProcessManager()
        rm = ROSLaunchManager()

        try:
            # Launch Heron
            rm.start(
                "heron",
                "heron_gazebo",
                "spawn_heron.launch",
                args={
                    "mass": mass,
                    "cog_y": cog_y,
                    "mi_z": mi_z,
                    "rdl": rdl,
                    "rdq": rdq,
                    "sdl": sdl,
                    "sdq": sdq,
                },
                sleep_time=5
            )

            # Start Gazebo
            rm.start(
                "gazebo",
                "heron_gazebo",
                "heron_world.launch",
                args={"world_name": "empty", "gui": "false"},
                sleep_time=15
            )

            # Start RViz
            pm.start("rviz", "rviz -d ~/.rviz/kingfisher-sim.rviz", sleep_time=5)

            # Start environment nodes
            rm.start(
                "environment_nodes",
                "kingfisher_experiments",
                "experiment_environment.launch",
                args={
                    "max_goal_distance": "100.0",
                    "odom_topic": "/pose_gt",
                    "loe_right": loe_right,
                    "pixel_noise_radius": pixel_noise_radius,
                    "distance_threshold": dst_thres,
                    "localization_delay": localization_delay,
                },
                sleep_time=10
            )

            # Enable perception noise
            if pixel_noise_radius > 0:
                rosservice_call("/goal_mux/select", "/waste_detector/goal")
            else:
                rosservice_call("/goal_mux/select", "/move_base_simple/goal")

            # Start RL agent
            rm.start(
                "rl_agent",
                "kingfisher_rl",
                "capture_agent_onnx.launch",
                args={"onnx_path": onnx_path},
                sleep_time=10
            )

            # Start bag recorder
            rm.start(
                "bag_recorder",
                "kingfisher_experiments",
                "bag_recorder.launch",
                args={"base_dir": base_dir, "exp_name": exp_name},
                sleep_time=2
            )

            # Run experiments
            rm.start(
                "experiment_runner",
                "kingfisher_experiments",
                "experiment_runner.launch",
                args={
                    "non_stop": "true",
                    "exp_file": exp_file,
                    "distance_threshold": dst_thres,
                    "exp_name": exp_name,
                    "stable_start_time": "2.0",
                    "vel_threshold": "0.05",
                }
            )

            print("Experiment started.")
            experiment_runner_launch = rm.launches.get("experiment_runner")
            if experiment_runner_launch:
                try:
                    print("Waiting for experiment to complete...")
                    experiment_runner_launch.spin()  # Blocks until shutdown
                    print("Experiment completed.")
                except KeyboardInterrupt:
                    print("Experiment interrupted by user.")
                except Exception as e:
                    print(f"Experiment ended: {e}")

            # Stop bag recorder
            rm.stop("bag_recorder")

            # Stop RL agent
            rm.stop("rl_agent")

            # Kill Heron and environment
            rm.stop("heron")
            rosservice_call("/gazebo/delete_model", "heron")
            rosservice_call("/gazebo/reset_world")
            rm.stop("environment_nodes")
            pm.stop("rviz")
            rm.stop("gazebo")

        except KeyboardInterrupt:
            print("Interrupted by user, shutting down...")
        finally:
            # Ensure all processes are stopped
            for name in list(rm.launches.keys()):
                print(f"Stopping {name}...")
                rm.stop(name)
            for name in list(pm.processes.keys()):
                print(f"Stopping {name}...")
                pm.stop(name)
            print("All processes terminated.")

if __name__ == "__main__":
    main()