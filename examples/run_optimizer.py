import argparse
import os
import yaml
import shutil
import sys

from calibrate_fk.parameter_optimizer import ParameterOptimizer


def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--urdf-file", "-u", help="Path to the URDF file")
    argument_parser.add_argument("--calibrate-on", "-e", help="Specify the folder to evaluate the model on")
    argument_parser.add_argument("--output-folder", "-o", help="Output folder for the results", default="output")
    argument_parser.add_argument("--variance", "-v", help="Variance of the noise", default=0.00)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="base_link")
    argument_parser.add_argument("--robot", "-r", help="Robot type", default="panda")
    argument_parser.add_argument("--overwrite", "-w", help="Overwrite the output folder", action="store_true")
    argument_parser.add_argument("--steps", "-s", help="Saving intermediate results", action="store_true")
    argument_parser.add_argument("--number-samples", "-n", help="Number of samples to use", default=None)
    argument_parser.add_argument("--offset-distance", "-d", help="Distance to offset the end effector", default=0.05)
    argument_parser.add_argument("--regularizer", "-reg", help="Regularizer for the optimization", default=1e-4)



    args = argument_parser.parse_args()
    urdf_file = args.urdf_file
    data_folder = args.calibrate_on
    variance = float(args.variance)
    end_effector = args.end_effector
    root_link = args.root_link
    output_folder = args.output_folder
    robot_name = args.robot
    overwrite = args.overwrite
    saving_steps = args.steps
    number_samples = args.number_samples
    offset_distance = float(args.offset_distance)
    regularizer = float(args.regularizer)
    if number_samples is not None:
        number_samples = int(number_samples)



    if output_folder == "output" or overwrite:
        shutil.rmtree(output_folder, ignore_errors=True)

    if os.path.exists(output_folder):
        print(f"Output folder {output_folder} already exists. Please delete it or specify a different folder.")
        sys.exit(1)
    os.makedirs(f"{output_folder}/images", exist_ok=True)
    config = {
            'urdf_file': urdf_file,
            'data_folder': data_folder,
            'variance': variance,
            'end_effector': end_effector,
            'root_link': root_link,
            'output_folder': output_folder,
            'robot_name': robot_name,
            'overwrite': overwrite,
            'saving_steps': saving_steps,
            'number_samples': number_samples,
            'offset_distance': offset_distance,
            'regularizer': regularizer,
            }
    with open(f"{output_folder}/config.yaml", "w") as f:
        yaml.dump(config, f)


    optimizer = ParameterOptimizer(output_folder)
    optimizer.set_offset_distance(offset_distance)
    optimizer.set_regulizer_weight(regularizer)
    optimizer.load_model(urdf_file)
    optimizer.read_data(data_folder, number_samples=number_samples)
    optimizer.create_symbolic_fk(root_link, end_effector)

    parameters = {
            'panda': [f"panda_joint{i}" for i in range(1, 8)] + ['ball_joint'],
            'panda_joints': [f"panda_joint{i}" for i in range(1, 8)],
            'iiwa14': [f"joint_a{i}" for i in range(1, 8)] + ['ball_joint'],
            'gen3lite': [f"joint_{i}" for i in range(1, 7)] + ['ball_joint'],
            'vx300': ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", ] + ['ball_joint'],
            'vx300s': ["waist", "shoulder", "forearm_roll", "elbow", "wrist_angle", "wrist_rotate", ] + ['ball_joint'],
            }
    #optimizer.select_parameters(variance=variance, selected_parameters=panda_parameters)
    optimizer.select_parameters(variance=variance, selected_parameters=parameters[robot_name])
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize(saving_steps=saving_steps)
    optimizer.evaluate_fks(verbose=True)


if __name__ == "__main__":
    main()
