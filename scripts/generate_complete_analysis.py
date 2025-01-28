import argparse
import os

def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", help="Data that you want to train the calibration on. Stored in data folder.")


    args = argument_parser.parse_args()
    model = args.model
    data = args.data
    robot_name = os.path.dirname(data)
    # run the script run_optimizer.py with the arguments model and data
    os.system(f"python3 run_optimizer.py --model {model} --data {data}")
    os.system(f"python3 generate_learning_curve.py --model {robot_name} ")
    os.system(f"python3 generate_overlay.py --model {robot_name} --data {data} ")
    os.system(f"python3 compute_improved_performance.py --model {robot_name} ")

if __name__ == "__main__":
    main()
