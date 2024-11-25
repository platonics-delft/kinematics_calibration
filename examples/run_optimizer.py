import argparse
import os
import shutil
import sys

from calibrate_fk.parameter_optimizer import ParameterOptimizer


def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--urdf-file", "-u", help="Path to the URDF file")
    argument_parser.add_argument("--calibrate-on", "-e", help="Specify the folder to evaluate the model on")
    argument_parser.add_argument("--output-folder", "-o", help="Output folder for the results", default="output")
    argument_parser.add_argument("--variance", "-v", help="Variance of the noise", default=0.01)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="base_link")
    argument_parser.add_argument("--robot", "-r", help="Robot type", default="panda")
    argument_parser.add_argument("--overwrite", "-w", help="Overwrite the output folder", action="store_true")
    argument_parser.add_argument("--steps", "-s", help="Saving intermediate results", action="store_true")


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


    if output_folder == "output" or overwrite:
        shutil.rmtree(output_folder, ignore_errors=True)

    if os.path.exists(output_folder):
        print(f"Output folder {output_folder} already exists. Please delete it or specify a different folder.")
        sys.exit(1)
    os.makedirs(f"{output_folder}/images", exist_ok=True)


    optimizer = ParameterOptimizer(output_folder)
    optimizer.load_model(urdf_file)
    optimizer.read_data(data_folder)
    optimizer.create_symbolic_fk(root_link, end_effector)

    parameters = {
            'panda': [f"panda_joint{i}" for i in range(1, 8)] + ['ball_joint'],
            'panda_joints': [f"panda_joint{i}" for i in range(1, 8)],
            'iiwa14': [f"joint_a{i}" for i in range(1, 8)] + ['ball_joint'],
            'gen3lite': [f"joint_{i}" for i in range(1, 7)] + ['ball_joint'],
            }
    #optimizer.select_parameters(variance=variance, selected_parameters=panda_parameters)
    optimizer.select_parameters(variance=variance, selected_parameters=parameters[robot_name])
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize(saving_steps=saving_steps)
    optimizer.evaluate_fks(verbose=True)


if __name__ == "__main__":
    main()
