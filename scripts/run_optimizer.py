import argparse
import os
import yaml
import shutil
import sys

from calibrate_fk.parameter_optimizer import ParameterOptimizer


def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", help="Data that you want to train the calibration on. Stored in data folder.")
    argument_parser.add_argument("--offset-distance", "-d", help="Distance to offset the end effector", default=0.05)
    argument_parser.add_argument("--regularizer", "-reg", help="Regularizer for the optimization", default=1e-4)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="base_link")

    argument_parser.add_argument("--variance_noise", "-v", help="Variance of the noise injected to the initial robot parameters", default=0.00)
    argument_parser.add_argument("--number-samples", "-n", help="Number of samples to use", default=None)
    #argument_parser.add_argument("--robot", "-r", help="Robot type", default="panda")
    #argument_parser.add_argument("--overwrite", "-w", help="Overwrite the output folder", action="store_true")
    #argument_parser.add_argument("--steps", "-s", help="Saving intermediate results", action="store_true")


    args = argument_parser.parse_args()
    model = args.model
    data = args.data
    robot_name = os.path.dirname(data)
    
    variance_noise = float(args.variance_noise)
    end_effector = args.end_effector
    root_link = args.root_link

    #overwrite = args.overwrite
    # saving_steps = args.steps
    overwrite = True
    saving_steps = True
    number_samples = args.number_samples
    offset_distance = float(args.offset_distance)
    regularizer = float(args.regularizer)
    if number_samples is not None:
        number_samples = int(number_samples)

    script_directory = os.path.abspath(__file__)

    # Get the parent directory of the script's directory
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    
    
    output_path = os.path.abspath(os.path.join(parent_directory, 'calibrated_urdf', robot_name))
    data_path = os.path.abspath(os.path.join(parent_directory, 'data', data))
    urdf_path = os.path.abspath(os.path.join(parent_directory, 'urdf', model + ".urdf"))

    
    if overwrite:
        shutil.rmtree(output_path, ignore_errors=True)

    if os.path.exists(output_path):
        print(f"Output folder {output_path} already exists. Please delete it or specify a different folder.")
        sys.exit(1)
    os.makedirs(f"{output_path}/images", exist_ok=True)
    config = {
            'urdf': model,
            'robot-name': robot_name,
            'variance_noise': variance_noise,
            'end_effector': end_effector,
            'root_link': root_link,
            'robot_name': robot_name,
            'overwrite': overwrite,
            'saving_steps': saving_steps,
            'number_samples': number_samples,
            'offset_distance': offset_distance,
            'regularizer': regularizer,
            'data_path': data_path,
            }
    with open(f"{output_path}/config.yaml", "w") as f:
        yaml.dump(config, f)

    print(output_path)
    optimizer = ParameterOptimizer(output_path)
    optimizer.set_offset_distance(offset_distance)
    optimizer.set_regulizer_weight(regularizer)
    optimizer.load_model(urdf_path)
    optimizer.read_data(data_path, number_samples=number_samples)
    optimizer.create_symbolic_fk(root_link, end_effector)

    parameters = {
            'panda': [f"panda_joint{i}" for i in range(1, 8)] + ['ball_joint'],
            'panda_joints': [f"panda_joint{i}" for i in range(1, 8)],
            'iiwa14': [f"joint_a{i}" for i in range(1, 8)] + ['ball_joint'],
            'gen3lite': [f"joint_{i}" for i in range(1, 7)] + ['ball_joint'],
            'vx300': ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", ] + ['ball_joint'],
            'vx300s': ["waist", "shoulder", "forearm_roll", "elbow", "wrist_angle", "wrist_rotate", ] + ['ball_joint'],
            }
    optimizer.select_parameters(variance_noise=variance_noise, selected_parameters=parameters[model])
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize(saving_steps=saving_steps)
    optimizer.evaluate_fks(verbose=True)


if __name__ == "__main__":
    main()
