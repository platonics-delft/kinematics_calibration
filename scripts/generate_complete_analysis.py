import argparse
import os
import sys
def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", help="Data that you want to train the calibration on. Stored in data folder.")


    args = argument_parser.parse_args()
    model = args.model
    data = args.data
    robot_name = os.path.dirname(data)

    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    urdf_path = parent_directory + "/urdf/" + model + ".urdf"
    if not os.path.exists(urdf_path):
        filename = os.path.splitext(os.path.basename(urdf_path))[0]
        print(f"URDF file {filename} does not exist.")
        # show the available model that can be used in from the folder urdf
        print("Available models are: ")
        for file in os.listdir(os.path.abspath(os.path.join(parent_directory, 'urdf'))):
            filename = os.path.splitext(file)[0]  # Get name without extension
            print(filename)
        sys.exit(1)

    data_path = os.path.abspath(os.path.join(parent_directory, 'data', data))

    if not os.path.exists(data_path):
        print(f"Data folder {data} does not exist in the data folder.")
        # Check if the data folder exists and the urdf path exists, otherwise exit
        data_root = os.path.abspath(os.path.join(parent_directory, 'data', model))
        print("Suggested data folders are: ")
        # List first level directories
        for d in sorted(os.listdir(data_root)):
            d_path = os.path.join(data_root, d)
            if os.path.isdir(d_path):
                print(f"{model}/{d}")
                # List second level directories
        sys.exit(1)


    # run the script run_optimizer.py with the arguments model and data
    os.system(f"python3 run_optimizer.py --model {model} --data {data}")
    os.system(f"python3 generate_learning_curve.py --model {robot_name} ")
    os.system(f"python3 generate_overlay.py --model {robot_name} --data {data} ")
    os.system(f"python3 compute_improved_performance.py --model {robot_name} ")

if __name__ == "__main__":
    main()
