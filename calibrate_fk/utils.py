import os
import xml.etree.ElementTree as ET
from tkinter import filedialog
import yaml
from typing import Optional, Tuple, List, Dict

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from yourdfpy import URDF

from io import BytesIO
import sys

def read_data(folder: str, number_samples: Optional[int] = None) -> None:
    # create an empty dictionary to store the data
    data = {} 
    for folder_ith in folder:
        robot_name = os.path.basename(folder_ith)
        data[robot_name] = read_each(folder_ith)
        if number_samples is not None:
            for i in range(len(data[robot_name])):
                data[robot_name][i] = data[robot_name][i][np.random.choice(data[robot_name][i].shape[0], number_samples, replace=False)]
    return data
def read_each(folder: Optional[str] = None) -> Tuple[np.ndarray, np.ndarray]:

    recording_folder = folder
    if not recording_folder or not os.path.exists(recording_folder):
        raise FileNotFoundError(f"Recording folder {recording_folder} not found.")
    file_path_1 = os.path.join(recording_folder, "hole_0.csv")
    if not os.path.exists(file_path_1):
        raise FileNotFoundError(f"Data file {file_path_1} not found.")
    file_path_2 = os.path.join(recording_folder, "hole_1.csv")
    if not os.path.exists(file_path_2):
        raise FileNotFoundError(f"Data file {file_path_2} not found.")
    data_1 = np.loadtxt(file_path_1, delimiter=",")
    data_2 = np.loadtxt(file_path_2, delimiter=",")
    return data_1, data_2

def replace_mesh_with_cylinder(urdf_file, out_file: str) -> str:
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Iterate through all links and replace meshes with cylinders
    for link in root.findall(".//link"):
        for visual in link.findall(".//visual"):
            geometry = visual.find("geometry")
            if geometry is not None:
                mesh = geometry.find("mesh")
                if mesh is not None:
                    # Replace <mesh> with <cylinder>
                    # Extract the mesh attributes (if needed for cylinder size)
                    radius = 0.05  # Default radius for the cylinder (adjust as needed)
                    length = 0.1   # Default length for the cylinder (adjust as needed)

                    # Create a new cylinder element
                    cylinder = ET.Element("cylinder")
                    cylinder.set("radius", str(radius))
                    cylinder.set("length", str(length))

                    # Replace the mesh element with the cylinder element
                    geometry.remove(mesh)
                    geometry.append(cylinder)

    # Save the modified URDF to a new file
    tree.write(out_file)
    print(f"Modified URDF saved as: {out_file}")
    return out_file

def overlay_images(images: list , output_path: str):
    images = [Image.open(BytesIO(img)) for img in images]
    # convert images into numpy arrays
    images = [np.array(img.convert("RGBA")).astype(np.float32) for img in images]
    width, height = images[0].shape[1], images[0].shape[0]
    assert all(img.shape[0] == height and img.shape[1] == width for img in images), "All images must have the same dimensions!"

    average_image = sum(images) / len(images)
    average_image = np.clip(average_image, 0, 255).astype(np.uint8)
    result_image = Image.fromarray(average_image, mode="RGBA")

    result_image.save(output_path)
    # show the image
    result_image.show()

def overlay_images_2(images: list, output_path: str):
    from rembg import remove

    """
    Overlays multiple images after removing their backgrounds and saves the result.
    """
    # Remove the background from all images using rembg
    images = [remove(img) for img in images]
    images = [Image.open(BytesIO(img)) for img in images]
    # Assuming 'images' is a list of Image objects with transparent backgrounds (RGBA)
    overlay = Image.new("RGBA", images[0].size, (0, 0, 0, 0))  # Create a blank image with the same size

    # Blend with transparency
    for img in images:
        # Composite
        overlay = Image.alpha_composite(overlay, img)

    # Optionally save the result
    overlay.save(output_path)
    overlay.show()

def plot_fks_iterations(points_list: list) -> None:
    colors = ['black', 'red', 'green', 'blue', 'yellow', 'purple', 'orange', 'cyan', 'magenta', 'brown']

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    # Generate unique colors for each list

    # Plot each list of points with its own color
    for i, points in enumerate(points_list):
        color = colors[i]
        for point in points:
            x, y, z = point
            # Plo the 3D scater
            ax.scatter(x, y, z, color=color)
        # Plot teh projection on the xyz


    # make axis equal
    limits = set_axes_equal(ax)



    """
    for i, points in enumerate(points_list):
        color = colors[i]
        for point in points:
            x, y, z = point
            # Plo the 3D scater
            ax.scatter(x, y, limits[2][0]+0.1, color=color, alpha=0.3)
            ax.scatter(x, limits[1][1]-0.1, z, color=color, alpha=0.3)
            ax.scatter(limits[0][0]+0.1, y, z, color=color, alpha=0.3)
    """,


# Ensure the same unit scale in all dimensions
def set_axes_equal(ax) -> np.ndarray:
    """Set equal scaling for 3D axes."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    ranges = limits[:, 1] - limits[:, 0]
    max_range = ranges.max()
    midpoints = limits.mean(axis=1)

    ax.set_xlim3d([midpoints[0] - max_range / 2, midpoints[0] + max_range / 2])
    ax.set_ylim3d([midpoints[1] - max_range / 2, midpoints[1] + max_range / 2])
    ax.set_zlim3d([midpoints[2] - max_range / 2, midpoints[2] + max_range / 2])
    return limits

def evaluate_model(model: URDF, data_folder: List[str], verbose: bool = False, offset_distance: float = 0.05) -> Dict[str, np.ndarray]:
    data = read_data(data_folder)
    statistics = {}
    for tool_position, recorded_joints in data.items():
        data_hole_0 = recorded_joints[0]
        data_hole_1 = recorded_joints[1]
        fks_1 = np.zeros((len(data_hole_0), 3))
        fks_2 = np.zeros((len(data_hole_1), 3))
        for i, joint_angles in enumerate(data_hole_0):
            joint= np.zeros(model.num_actuated_joints)
            joint[:len(joint_angles)] = joint_angles
            model.update_cfg(joint)
            fk = model.get_transform(frame_from=model.base_link, frame_to="ball_link")
            fks_1[i] = np.array(fk[:3, 3]).flatten()
        for i, joint_angles in enumerate(data_hole_1):
            joint= np.zeros(model.num_actuated_joints)
            joint[:len(joint_angles)] = joint_angles
            model.update_cfg(joint)
            fk = model.get_transform(frame_from=model.base_link, frame_to="ball_link")
            fks_2[i] = np.array(fk[:3, 3]).flatten()


        fk_mean_1 = np.mean(fks_1, axis=0)
        N_1 = len(fks_1)
        fk_mean_2 = np.mean(fks_2, axis=0)
        N_2 = len(fks_2)
        fk_variance_1 = np.sum(np.var(fks_1, axis=0))
        fk_variance_2 = np.sum(np.var(fks_2, axis=0))
        distance_error = np.abs(np.linalg.norm(fk_mean_1 - fk_mean_2) - offset_distance)
        # average_absolute_error 
        fks_1_average = np.mean(np.linalg.norm(np.abs(fks_1 - fk_mean_1), axis=1))  
        fks_2_average = np.mean(np.linalg.norm(np.abs(fks_2 - fk_mean_2), axis=1))
        weighted_average = (N_1 * fks_1_average + N_2 * fks_2_average)/(N_1 + N_2)
        std_dev_fk = np.sqrt(fk_variance_1 + fk_variance_2) # this is not the actual error 
        mean_squared_error = (fk_variance_1  * N_1 + fk_variance_2 * N_2)/(N_1+N_2)
        std_dev_fk = np.sqrt(mean_squared_error) 
        statistics[tool_position] = {
            "mean_1": fk_mean_1.tolist(),
            "std_dev_1": float(np.sqrt(fk_variance_1)),
            "mean_2": fk_mean_2.tolist(),
            "std_dev_2": float(np.sqrt(fk_variance_2)),
            "distance_error": float(distance_error),
            "fks_1": fks_1.tolist(),
            "fks_2": fks_2.tolist(),
            "mean_absolute_error": float(weighted_average),
            "std_dev_fk": float(std_dev_fk),
        }
        
        if verbose: 
            print(f"Tool position: {tool_position}")
            print(f"Mean 1: {fk_mean_1}")
            print(f"Mean 2: {fk_mean_2}")
            print(f"Distance error: {distance_error}")
            print(f"Mean absolute error: {weighted_average}")
            print(f"Standard deviation: {std_dev_fk}")

    return statistics


def compute_improved_performance(model_folder: str, data_folders_train: List[str], data_folders_test: List[str], offset_distance, latex=False) -> None:

    kpis = yaml.load(open(f"{model_folder}/kpis.yaml"), Loader=yaml.FullLoader)
    nb_steps = kpis["nb_steps"]

    steps = list(range(nb_steps))
    steps = [0, nb_steps ]
    distances_train = [[] for _ in data_folders_train]
    distances_test = [[] for _ in data_folders_test]

    variances_train =  [[] for _ in data_folders_train]
    variances_test = [[] for _ in data_folders_test]

    mae_train = [[] for _ in data_folders_train]
    mae_test = [[] for _ in data_folders_test]
    print(f"Offset distance: {offset_distance}")

    for step in steps:
        print(f"Step {step}")
        model_step = URDF.load(f"{model_folder}/step_{step}/model.urdf")

        statistics_train = evaluate_model(model_step, data_folders_train, offset_distance=offset_distance)
        for i, (key, value) in enumerate(statistics_train.items()):
            distance_error = value["distance_error"]
            mae = value["mean_absolute_error"]
            std_dev = value["std_dev_fk"]
            distances_train[i].append(distance_error)
            variances_train[i].append(std_dev)
            mae_train[i].append(mae)

        statistics_test = evaluate_model(model_step, data_folders_test, offset_distance=offset_distance)

        for i, (key, value) in enumerate(statistics_test.items()):
            distance_error = value["distance_error"]
            mae = value["mean_absolute_error"]
            std_dev = value["std_dev_fk"]
            distances_test[i].append(distance_error)
            variances_test[i].append(std_dev)
            mae_test[i].append(mae)
        key_test = list(value.keys())
    percentage_improved_distance_train =  [(distances_train[i][0] - distances_train[i][-1])/distances_train[i][0] * 100 for i in range(len(data_folders_train))]
    percentage_improved_variance_train =  [(variances_train[i][0] - variances_train[i][-1])/variances_train[i][0] * 100 for i in range(len(data_folders_train))]
    percentage_improved_mae_train =  [(mae_train[i][0] - mae_train[i][-1])/mae_train[i][0] * 100 for i in range(len(data_folders_train))]
    print (f"The mean absolute error  went from {np.mean(np.array(mae_train)[:,0]):.2e} to {np.mean(np.array(mae_train)[:,-1]):.2e} on the train data on average")
    # print (f"The consistency went from {np.mean(np.array(variances_train)[:,0]):.2e} to {np.mean(np.array(variances_train)[:,-1]):.2e} on the train data on average")
    # print (f"The distortion went from {np.mean(np.array(distances_train)[:,0]):.2e} to {np.mean(np.array(distances_train)[:,-1]):.2e} on the train data on average")
    print(f"Percentage of removed error on train set: {np.mean(percentage_improved_mae_train):.2f}")
    # print(f"Percentage of removed error on train set: {np.mean(percentage_improved_variance_train)}")
    # print(f"Percentage of removed distortion error on train set {np.mean(percentage_improved_distance_train)}")


    if distances_test: # if there is test data
        percentage_improved_distance_test =  [(distances_test[i][0] - distances_test[i][-1])/distances_test[i][0] * 100 for i in range(len(data_folders_test))]
        percentage_improved_variance_test =  [(variances_test[i][0] - variances_test[i][-1])/variances_test[i][0] * 100 for i in range(len(data_folders_test))]
        percentage_improved_mae_test =  [(mae_test[i][0] - mae_test[i][-1])/mae_test[i][0] * 100 for i in range(len(data_folders_test))]

        
        print (f"The mean absolute error  went from {np.mean(np.array(mae_test)[:,0]):.2e} to {np.mean(np.array(mae_test)[:,-1]):.2e} on the test data on average")
        # print (f"The consistency went from {np.mean(np.array(variances_test)[:,0]):.2e} to {np.mean(np.array(variances_test)[:,-1]):.2e} on the test data on average")
        # print (f"The distortion went from {np.mean(np.array(distances_test)[:,0]):.2e} to {np.mean(np.array(distances_test)[:,-1]):.2e} on the test data on average")
        
        print(f"Percentage of removed error on test set: {np.mean(percentage_improved_mae_test):.2f}")
        # print(f"Percentage of removed error on test set: {np.mean(percentage_improved_variance_test)}")
        # print(f"Percentage of removed distortion error on test set {np.mean(percentage_improved_distance_test)}")
    else:
        print("No test data provided")
def plot_training_curves(model_folder: str, data_folders_train: str, data_folders_test: List[str], offset_distance, repeatability : float, latex=False) -> None:
    if latex == True:
        plt.rcParams.update({
        "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
        "text.usetex": False,          # Use LaTeX for text rendering
        "font.family": "serif",       # Use a serif font
        "pgf.rcfonts": False,         # Disable using Matplotlib's default font settings
        })
    kpis = yaml.load(open(f"{model_folder}/kpis.yaml"), Loader=yaml.FullLoader)
    nb_steps = kpis["nb_steps"]
    steps = list(range(nb_steps+1))
    distances_train = [[] for _ in data_folders_train]
    distances_test = [[] for _ in data_folders_test]
    variances_train = [[] for _ in data_folders_train]
    variances_test = [[] for _ in data_folders_test]
    print(f"Offset distance: {offset_distance}")

    for step in steps:
        print(f"Step {step}")
        model_step = URDF.load(f"{model_folder}/step_{step}/model.urdf")
        statistics = evaluate_model(model_step, data_folders_train, offset_distance=offset_distance)
        for i, (key, value) in enumerate(statistics.items()):
            distance_error = value["distance_error"]
            std_dev = value["std_dev_fk"]
            distances_train[i].append(distance_error)
            variances_train[i].append(std_dev)
        key_training = list(statistics.keys())

        statistics = evaluate_model(model_step, data_folders_test, offset_distance=offset_distance)
        for i, (key, value) in enumerate(statistics.items()):
            distance_error = value["distance_error"]
            std_dev = value["std_dev_fk"]
            distances_test[i].append(distance_error)
            variances_test[i].append(std_dev)
        key_test = list(statistics.keys())
    # breakpoint()
    fontsize=20
    # CONSISTENCY
    fig, ax = plt.subplots(1, 1)

    ax.set_yscale("log")
    for i, variances in enumerate(variances_train):
        if i == 0:
            ax.plot(steps, variances, color= 'blue', linewidth=3, label="Train")
        else:
            ax.plot(steps, variances, color= 'blue', linewidth=3)
    for i, variances in enumerate(variances_test):
        if i == 0:
            ax.plot(steps, variances, color= 'orange', linewidth=3, label="Test")
        else:
            ax.plot(steps, variances, color= 'orange', linewidth=3)

    ax.legend(fontsize=fontsize)
    ax.set_xlabel("Step", fontsize=fontsize)
    ax.set_ylabel("$\sigma$ [m]", fontsize=fontsize)
    ax.axhline(y=repeatability, color='k', linestyle='--', linewidth=3)

    ax.tick_params(axis='both', which='major', labelsize=fontsize)
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.set_ylim(1e-4/2, 3e-2)

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    if latex:
        plt.savefig(f"{model_folder}/consistency.pgf")
    else:
        plt.savefig(f"{model_folder}/consistency.png")



    # DISTORTION
        fig, ax = plt.subplots(1, 1)
    # make log scale
    ax.set_yscale("log")
    for i, distances in enumerate(distances_train):
        if i == 0:
            ax.plot(steps, distances, color='blue', linewidth=3, label="Train")
        else:
            ax.plot(steps, distances, color='blue', linewidth=3)
    for i, distances in enumerate(distances_test):
        name = data_folders_test[i].split("/")[-1]

        if i == 0:
            ax.plot(steps, distances, color='orange', linewidth=3, label="Test")
        else:
            ax.plot(steps, distances, color='orange', linewidth=3)
    #plot dashed horixzaon line at the repeatibility value
    print(f"Repeatability: {repeatability}")
    ax.axhline(y=repeatability, color='k', linestyle='--', linewidth=3)
    # set legend
    ax.legend(fontsize=fontsize)
    ax.set_xlabel("Step", fontsize=fontsize)
    ax.set_ylabel("$\epsilon$ [m]", fontsize=fontsize)
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    # only show the step at every 5th
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))

    ax.set_ylim(1e-7, 1e-2)
    # ax.set_ylim(1e-4/2, 3e-2)

    # set fontsize of the x and y numbers
    ax.tick_params(axis='both', which='major', labelsize=fontsize)
    # set title

    # the figure should have tight margins but the legend should not be cut off
    # make size of figure suitable for journal paper IEEE standard
    # fig.set_size_inches(3.487/2, 1.5)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.tight_layout()
    if latex:
        plt.savefig(f"{model_folder}/distortion.pgf")
    else:
        plt.savefig(f"{model_folder}/distortion.png")


    plt.show()


    # Can you export each axes to a separate file?


import re

def modify_urdf(urdf_file: str, search_pattern: str, replace_with: str, output_file: str):
    """
    Load the URDF file, search for a specific line/element, replace it, and save the new URDF.

    Args:
        urdf_file (str): Path to the input URDF file.
        search_pattern (str): The line or pattern to search for.
        replace_with (str): The text to replace the found line with.
        output_file (str): Path to the new URDF file where the modified content will be saved.
    """
    
    # Read the original URDF file
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    # Search and replace the content
    urdf_content_modified = re.sub(search_pattern, replace_with, urdf_content)

    # Write the modified content to a new URDF file
    with open(output_file, 'w') as file:
        file.write(urdf_content_modified)

    print(f"Modified URDF saved to: {output_file}")

def check_urdf_path(urdf_path : str):
    if not os.path.exists(urdf_path):
        filename = os.path.splitext(os.path.basename(urdf_path))[0]
        print(f"URDF file {filename} does not exist.")
        # show the available model that can be used in from the folder urdf
        print("Available models are: ")
        script_directory = os.path.abspath(__file__)
        parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
        for file in os.listdir(os.path.abspath(os.path.join(parent_directory, 'urdf'))):
            filename = os.path.splitext(file)[0]  # Get name without extension
            print(filename)
        sys.exit(1)
def check_data_path(data_path : List[str]):
    for data_path_ith in data_path:
        if not os.path.exists(data_path_ith):
            print(f"Data folder {data_path_ith} does not exist.")
            script_directory = os.path.abspath(__file__)
            parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
            # Check if the data folder exists and the urdf path exists, otherwise exit
            data_root = os.path.abspath(os.path.join(parent_directory, 'data'))
            print("Available data folders are: ")
            # List first level directories
            for d in sorted(os.listdir(data_root)):
                d_path = os.path.join(data_root, d)
                if os.path.isdir(d_path):
                    print(f"    {d}/")
                    # List second level directories
                    for subd in os.listdir(d_path):
                        subd_path = os.path.join(d_path, subd)
                        if os.path.isdir(subd_path):
                            print(f"        {subd}")
            sys.exit(1)

def check_model_path(model_folder: str):
    if not os.path.exists(model_folder):
        # read last directory in model_folder
        model = os.path.basename(model_folder)
        print(f"model {model} does not exist in calibrated_urdf. Did you already run the optimizer?")
        # Check if the data folder exists and the urdf path exists, otherwise exit
        script_directory = os.path.abspath(__file__)
        parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
        data_root = os.path.abspath(os.path.join(parent_directory, 'data'))
        print("Available calibrated models that can be evaluated are: ")
        # List first level directories
        for d in sorted(os.listdir(data_root)):
            if d == "README.md":
                continue
            print(f"    {d}")
        sys.exit(1)
