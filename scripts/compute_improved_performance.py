import argparse
import os
import yaml
from calibrate_fk.utils import compute_improved_performance
import sys

def main():
    parser = argparse.ArgumentParser(description='Create plots for the calibration data')
    parser.add_argument('--model', "-m", type=str, help='Calibrated robot model. Saved in the calibrated_urdf folder.')
    parser.add_argument('--latex', "-l", action='store_true', help='Use latex for saving of the plots as .pgf. If not they are saved as png.')
    args = parser.parse_args()

    model = args.model
    latex = args.latex
    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    model_folder = parent_directory + "/calibrated_urdf/" + model


    if not os.path.exists(model_folder):
        print(f"model {model_folder} does not exist in calibrated_urdf. Did you already run the optimizer?")
        # Check if the data folder exists and the urdf path exists, otherwise exit
        data_root = os.path.abspath(os.path.join(parent_directory, 'data'))
        print("Available calibrated models that can be evaluated are: ")
        # List first level directories
        for d in sorted(os.listdir(data_root)):
            if d == "README.md":
                continue
            print(f"    {d}")
        sys.exit(1)


    model_config = yaml.load(open(model_folder + "/config.yaml", 'r'), Loader=yaml.FullLoader)
    offset_distance = model_config["offset_distance"]
    training_data = model_config["data_path"]
    data_folder = os.path.abspath(parent_directory + "/data/" + model)
    folders = [f.path for f in os.scandir(data_folder) if f.is_dir()]
    evaluation_data = [folder for folder in folders if folder not in training_data]

    print(f"Model folder: {model_folder}")
    print(f"Training data: {training_data}")
    print(f"Evaluation data: {evaluation_data}")

    compute_improved_performance(model_folder, training_data, evaluation_data, offset_distance, latex)

if __name__ == "__main__":
    main()







