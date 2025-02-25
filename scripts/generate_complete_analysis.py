import argparse
import os
import sys
from calibrate_fk.utils import check_urdf_path, check_data_path
def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")


    args = argument_parser.parse_args()
    model = args.model
    data = args.data
    robot_name = os.path.dirname(data[0])

    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    urdf_path = parent_directory + "/urdf/" + model + ".urdf"
    check_urdf_path(urdf_path)

    data_path = [None] * len(data)
    for i, d in enumerate(data):
        data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))
    check_data_path(data_path)

    # run the script run_optimizer.py with the arguments model and data
    data_args = ' '.join(data)
    os.system(f"python3 run_optimizer.py --model {model} --data {data_args}")
    os.system(f"python3 generate_learning_curve.py --model {robot_name} ")
    # os.system(f"python3 generate_overlay.py --model {robot_name} --data {data_args} ")
    os.system(f"python3 compute_improved_performance.py --model {robot_name} ")

if __name__ == "__main__":
    main()
