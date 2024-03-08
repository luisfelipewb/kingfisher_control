#!/usr/bin/python3
import os
import subprocess

directory = "/home/lwolfbat/uuv_ws/src/kingfisher_control/kingfisher_rl/output"

for filename in os.listdir(directory):
    if filename.endswith(".bag"):
        print(f"Extracting from {filename}...")

        bag_path = os.path.join(directory, filename)
        csv_path = bag_path.replace(".bag", ".csv")

        command = f"rosrun kingfisher_rl extract_capture.py -i {bag_path} -o {csv_path}"
        
        try:
            subprocess.run(command, shell=True, check=True)
            print(f"Processed {filename} successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Error processing {filename}: {e}")


for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        print(f"Ploting {filename}...")

        csv_path = os.path.join(directory, filename)
        plot_path = csv_path.replace(".csv", ".png")

        command = f"rosrun kingfisher_rl plot_capture.py -i {csv_path} -o {plot_path}"

        try:
            subprocess.run(command, shell=True, check=True)
            print(f"Plotted {filename} successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Error plotting {filename}: {e}")

print("Done.")