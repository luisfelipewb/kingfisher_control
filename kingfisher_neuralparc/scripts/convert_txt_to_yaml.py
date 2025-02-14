import yaml

def convert_text_to_yaml(input_file, output_file):
    experiments = []

    with open(input_file, 'r') as file:
        for line in file:
            dist, bearing = map(float, line.split())
            print(f"dist: {dist}, bearing: {bearing}")
            # convert bearing from rad to deg
            bearing = bearing * 180 / 3.14159
            experiment = {
                'dist': dist,
                'bearing': bearing,
                'v0': 0.0
            }
            experiments.append(experiment)

    data = {'experiments': experiments}

    with open(output_file, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

if __name__ == "__main__":
    input_file = '../config/seeds_local_rev2.txt'  # Replace with your input text file name
    output_file = '../config/seeds2.yaml'  # Replace with your desired output YAML file name
    convert_text_to_yaml(input_file, output_file)
    print(f"Converted {input_file} to {output_file}")