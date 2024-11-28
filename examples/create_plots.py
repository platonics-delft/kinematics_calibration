import argparse
import yaml
from calibrate_fk.utils import plot_distance_curves


def main():
    parser = argparse.ArgumentParser(description='Create plots for the calibration data')
    parser.add_argument('--model-folder', "-m", type=str, help='Path to model')
    parser.add_argument('--training-data', "-t", type=str, help='Path to training data')
    parser.add_argument('--evaluation-data', "-e", type=str, help='Path to evaluation data', nargs='+')
    args = parser.parse_args()

    model_folder = args.model_folder
    training_data = args.training_data
    evaluation_data = args.evaluation_data

    model_config = yaml.load(open(model_folder + "/config.yaml", 'r'), Loader=yaml.FullLoader)
    offset_distance = model_config["offset_distance"]

    print(f"Model folder: {model_folder}")
    print(f"Training data: {training_data}")
    print(f"Evaluation data: {evaluation_data}")

    plot_distance_curves(model_folder, training_data, evaluation_data, offset_distance)

if __name__ == "__main__":
    main()







