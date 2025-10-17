import argparse
import os
import yaml
import shutil
import sys

from calibrate_fk.parameter_optimizer import ParameterOptimizer
from calibrate_fk.utils import check_urdf_path, check_data_path, remove_outliers

def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")
    argument_parser.add_argument("--offset-distance", "-d", help="Distance between the sockets in meters, defauls it 0.05", default=0.05)
    argument_parser.add_argument("--regularizer", "-reg", help="Regularizer for the optimization", default=1e-4)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="base_link")
    argument_parser.add_argument("--variance_noise", "-v", help="Variance of the noise injected to the initial robot parameters", default=0.00)
    argument_parser.add_argument("--number-samples", "-n", help="Number of samples to use", default=None)

    args = argument_parser.parse_args()
    model = args.model
    data = args.data
    robot_name = os.path.dirname(data[0])
    variance_noise = float(args.variance_noise)
    end_effector = args.end_effector
    root_link = args.root_link

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

    # data_path = os.path.abspath(os.path.join(parent_directory, 'data', data))
    urdf_path = os.path.abspath(os.path.join(parent_directory, 'urdf', model + ".urdf"))

    check_urdf_path(urdf_path)

    data_path = [None] * len(data)
    for i, d in enumerate(data):
        data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))
    
    check_data_path(data_path)


    # Does the output folder already exist?
    print("Output path: ", output_path)
    print(os.path.exists(output_path))  
    if os.path.exists(output_path):
        shutil.rmtree(output_path)

    if os.path.exists(output_path):
        print(f"Output folder {output_path} already exists. Please delete it or specify a different folder.")
        sys.exit(1)
    os.makedirs(f"{output_path}", exist_ok=True)
    config = {
            'urdf': model,
            'robot-name': robot_name,
            'variance_noise': variance_noise,
            'end_effector': end_effector,
            'root_link': root_link,
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
    # check if datapath has folder inside or only files
    optimizer.read_data(data_path, number_samples=number_samples)   
    optimizer.create_symbolic_fk(root_link, end_effector)
    #import the yaml file named like model .yaml
    with open(f"{parent_directory}/config_optimizer/{model}.yaml", "r") as f:
        parameters = yaml.load(f, Loader=yaml.FullLoader)
    optimizer.select_parameters(variance_noise=variance_noise, selected_parameters=parameters['joints'])
    optimizer.data= remove_outliers(optimizer._model, optimizer.data)
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize(use_dynamic_means=True, use_distortion_error=False, use_regularization=False, saving_steps=saving_steps)
    optimizer.evaluate_fks(verbose=True)


if __name__ == "__main__":
    main()
