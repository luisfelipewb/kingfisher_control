import os
import pandas as pd
import numpy as np

def process_file(filepath):
    # Read the CSV file
    df = pd.read_csv(filepath)
    
    # Get the initial offsets
    initial_pos_x = df['pos_x'].iloc[0]
    initial_pos_y = df['pos_y'].iloc[0]
    initial_yaw = df['yaw'].iloc[0]
    
    # Remove the initial offsets
    df['pos_x'] -= initial_pos_x
    df['pos_y'] -= initial_pos_y
    # df['yaw'] -= initial_yaw
    
    # # Rotate the positions based on yaw 
    # # (assuming the robot starts at the origin facing the x-axis)
    # x = df['pos_x']
    # y = df['pos_y']
    # yaw = initial_yaw
    # df['pos_x'] = x*np.cos(yaw) + y*np.sin(yaw)
    # df['pos_y'] = -x*np.sin(yaw) + y*np.cos(yaw)

    # # Ensure yaw is between -pi and pi
    # df['yaw'] = (df['yaw'] + np.pi) % (2 * np.pi) - np.pi
    
    # Save the processed file
    df.to_csv(filepath, index=False)

def process_folder(folder):
    for file in os.listdir(folder):
        if file.endswith(".csv"):
            process_file(os.path.join(folder, file))

if __name__ == "__main__":
    folder_path = '../output/'  # Replace with the path to your folder
    # process all csv files in the folder
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            file_path = folder_path + file
            print(file_path)
            process_file(folder_path + file)
