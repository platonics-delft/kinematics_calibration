import shutil
import os
import sys
import argparse
from calibrate_fk.parameter_optimizer import ParameterOptimizer


def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--urdf_file", "-u", help="Path to the URDF file")
    argument_parser.add_argument("--calibrate-on", "-e", help="Specify the folder to evaluate the model on")
    argument_parser.add_argument("--output_folder", "-o", help="Output folder for the results", default="output")
    argument_parser.add_argument("--variance", "-v", help="Variance of the noise", default=0.01)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="link0")


    args = argument_parser.parse_args()
    urdf_file = args.urdf_file
    data_folder = args.calibrate_on
    variance = float(args.variance)
    end_effector = args.end_effector
    root_link = args.root_link
    output_folder = args.output_folder

    if output_folder == "output":
        shutil.rmtree(output_folder, ignore_errors=True)

    if os.path.exists(output_folder):
        print(f"Output folder {output_folder} already exists. Please delete it or specify a different folder.")
        sys.exit(1)
    os.makedirs(f"{output_folder}/images", exist_ok=True)


    optimizer = ParameterOptimizer(output_folder)
    optimizer.load_model(urdf_file)
    optimizer.read_data(data_folder)
    optimizer.create_symbolic_fk(root_link, end_effector)
    panda_parameters = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "ball_joint",
        ]
    iiwa14_parameters = [
            "joint_a1",
            "joint_a2",
            "joint_a3",
            "joint_a4",
            "joint_a5",
            "joint_a6",
            "joint_a7",
            "ball_joint",
        ]
    #optimizer.select_parameters(variance=variance, selected_parameters=panda_parameters)
    optimizer.select_parameters(variance=variance, selected_parameters=iiwa14_parameters)
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize()
    optimizer.evaluate_fks(verbose=True)


if __name__ == "__main__":
    main()
