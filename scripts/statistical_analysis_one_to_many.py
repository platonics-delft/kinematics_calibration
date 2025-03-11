import argparse
import os
import yaml
import matplotlib.pyplot as plt
from calibrate_fk.parameter_optimizer import ParameterOptimizer
from calibrate_fk.utils import check_urdf_path, check_data_path, evaluate_model
import matplotlib.colors as mcolors
from matplotlib.patches import Patch
def main():
    
    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.")
    argument_parser.add_argument("--data", "-t", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")
    argument_parser.add_argument("--offset-distance", "-d", help="Distance between the two sockets", default=0.05)
    argument_parser.add_argument("--regularizer", "-reg", help="Regularizer coefficient for the optimization", default=1e-4)
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

    saving_steps = False
    number_samples = args.number_samples
    offset_distance = float(args.offset_distance)
    regularizer = float(args.regularizer)
    if number_samples is not None:
        number_samples = int(number_samples)

    script_directory = os.path.abspath(__file__)
    # Get the parent directory of the script's directory
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    
    
    output_path = os.path.abspath(os.path.join(parent_directory, 'calibrated_urdf', robot_name))
    os.makedirs(f"{output_path}", exist_ok=True)
    urdf_path = os.path.abspath(os.path.join(parent_directory, 'urdf', model + ".urdf"))
    check_urdf_path(urdf_path)

    data_path = [None] * len(data)
    for i, d in enumerate(data):
        data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))
    
    check_data_path(data_path)

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

    optimizer = ParameterOptimizer(output_path)
    optimizer.set_offset_distance(offset_distance)
    optimizer.set_regulizer_weight(regularizer)
    optimizer.load_model(urdf_path)
    optimizer.create_symbolic_fk(root_link, end_effector)
    # check if datapath has folder inside or only file
    parameters = {
            'panda': [f"panda_joint{i}" for i in range(1, 8)] + ['ball_joint'],
            'iiwa14': [f"joint_a{i}" for i in range(1, 8)] + ['ball_joint'],
            'gen3lite': [f"joint_{i}" for i in range(1, 7)] + ['ball_joint'],
            'vx300s': ["waist", "shoulder", "forearm_roll", "elbow", "wrist_angle", "wrist_rotate", ] + ['ball_joint'],
            }
    optimizer.select_parameters(variance_noise=variance_noise, selected_parameters=parameters[model]) 
    
    max_mae = 0
    data_train = []
    data_test = []  
    for data_ in data_path:
        data_train.append([data_])
        #select the data in data_path that are not in data_path 
        other_data = [d for d in data_path if d != data_]
        data_test.append(other_data)
    #create a list of color, each for each data_train, the list may have a different lenght 
    
    # Dictionary of Tableau colors (always in same order)
    tableau_colors = mcolors.TABLEAU_COLORS

    # Convert to list if needed
    colors = list(tableau_colors.values())
    colors = colors[:len(data_train)]
    plt.figure()

    optimizer.read_data(data_train[0], number_samples=number_samples)
    optimizer.optimize(saving_steps=False)
    statistics= evaluate_model(optimizer._model, data_path, verbose=False) 
    stat = []   
    for values in statistics.values():
        stat.append(values["mean_absolute_error"]*1000)
    max_mae =max(max_mae, max(stat))
    keys = list(statistics.keys())
    key_train = [os.path.basename(train) for train in data_train[0]]
    # set y axis scale 
    # Combine the legends
    hatch_legend = Patch(facecolor='white', edgecolor='gray', hatch='//', label='Training')
    bars_legend = [Patch(facecolor=color, label=key) for color, key in zip(colors, keys)]
    plt.legend(handles=[hatch_legend] + bars_legend, loc='upper center', ncol=len(data_train), fontsize=20)

    hatch_styles = ['' if key not in key_train else '//' for key in keys]
    bars = plt.bar(keys, stat, color=colors)
    for bar, hatch in zip(bars, hatch_styles):
        bar.set_hatch(hatch)
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    
    plt.ylabel('Mean Absolute Error (mm)', fontsize=20)
    plt.savefig(f"{output_path}/bar_plot_one_to_many.png", bbox_inches='tight')
    plt.show()  
if __name__ == "__main__":
    main()
