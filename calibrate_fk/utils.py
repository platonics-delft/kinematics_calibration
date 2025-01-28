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


OFFSET_DISTANCE = 0.05

def read_data(folder: Optional[str] = None) -> Tuple[np.ndarray, np.ndarray]:
    if not folder:
        default_data_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../ros_ws")
        print("Select the folder containing the recorded data.")
        recording_folder = filedialog.askdirectory(
            title="Select a data folder.",
            initialdir=default_data_directory,
        )
    else:
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

def evaluate_model(robot_model: URDF, data_folder: str, verbose: bool = False, offset_distance: float=0.05) -> dict:

    data_hole_0, data_hole_1 = read_data(data_folder)


    fks_1 = np.zeros((len(data_hole_0), 3))
    fks_2 = np.zeros((len(data_hole_1), 3))
    for i, joint_angles in enumerate(data_hole_0):
        joints= np.zeros(robot_model.num_actuated_joints)   
        joints[:len(joint_angles)] = joint_angles
        robot_model.update_cfg(joints)
        fk = robot_model.get_transform(frame_from=robot_model.base_link, frame_to="ball_link")
        fks_1[i] = np.array(fk[:3, 3]).flatten()
    for i, joint_angles in enumerate(data_hole_1):
        joints= np.zeros(robot_model.num_actuated_joints)
        joints[:len(joint_angles)] = joint_angles
        robot_model.update_cfg(joints)
        fk = robot_model.get_transform(frame_from=robot_model.base_link, frame_to="ball_link")
        fks_2[i] = np.array(fk[:3, 3]).flatten()

    fk_mean_1 = np.round(np.mean(fks_1, axis=0), decimals=10)
    fk_variance_1 = np.round(np.var(fks_1, axis=0), decimals=10)
    fk_mean_2 = np.round(np.mean(fks_2, axis=0), decimals=10)
    fk_variance_2 = np.round(np.var(fks_2, axis=0), decimals=10)
    distance_error = np.round(np.linalg.norm(fk_mean_1 - fk_mean_2) - offset_distance, decimals=10)
    consistency = np.round(np.sqrt(np.sum(fk_variance_1 + fk_variance_2)), decimals=10)
    if verbose:
        print(f"Mean_1: {fk_mean_1}")
        print(f"Variance_1: {fk_variance_1}")
        print(f"Mean_2: {fk_mean_2}")
        print(f"Variance_2: {fk_variance_2}")
        print(f"Consistency: {consistency}")
        print(f"Distortion: {distance_error}")
        print(f"Height Error: {fk_mean_1[2] - fk_mean_2[2]}")
    kpis = {
        "mean_1": fk_mean_1.tolist(),
        "var_1": fk_variance_1.tolist(),
        "mean_2": fk_mean_2.tolist(),
        "var_2": fk_variance_2.tolist(),
        "distance": float(distance_error),
        "height": float(fk_mean_1[2] - fk_mean_2[2]),
        "fks_1": fks_1.tolist(),
        "fks_2": fks_2.tolist(),
    }
    return kpis

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

def compute_statistics(model: URDF, data_folder: str, offset_distance: float = 0.05) -> Dict[str, np.ndarray]:
    print(f"Data folder {data_folder}")
    data_hole_0, data_hole_1 = read_data(data_folder)

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
    fk_mean_2 = np.mean(fks_2, axis=0)
    fk_variance_1 = np.sum(np.var(fks_1, axis=0))
    fk_variance_2 = np.sum(np.var(fks_2, axis=0))
    average_z_1 = np.mean(fks_1[:, 2])
    average_z_2 = np.mean(fks_2[:, 2])
    distance_error = np.abs(np.linalg.norm(fk_mean_1 - fk_mean_2) - offset_distance)
    std_dev_fk = np.sqrt(fk_variance_1 + fk_variance_2)
    average_z = np.linalg.norm(average_z_1 - average_z_2)
    statistics = {
        "mean_1": fk_mean_1,
        "std_dev_1": np.sqrt(fk_variance_1),
        "mean_2": fk_mean_2,
        "std_dev_2": np.sqrt(fk_variance_2),
        "distance_error": float(distance_error),
        "height_error": average_z,
        "fks_1": fks_1,
        "fks_2": fks_2,
        "std_dev_fk": std_dev_fk,
    }
    return statistics


def compute_improved_performance(model_folder: str, data_folder_train: str, data_folders_test: List[str], offset_distance, latex=False) -> None:

    kpis = yaml.load(open(f"{model_folder}/kpis.yaml"), Loader=yaml.FullLoader)
    nb_steps = kpis["nb_steps"]

    steps = list(range(nb_steps))
    distances_train = []
    distances_test = [[] for _ in data_folders_test]
    variances_train = []
    variances_test = [[] for _ in data_folders_test]
    print(f"Offset distance: {offset_distance}")

    for step in steps:
        print(f"Step {step}")
        model_step = URDF.load(f"{model_folder}/step_{step}/model.urdf")
        statistics = compute_statistics(model_step, data_folder_train, offset_distance=offset_distance)
        distance_error = statistics["distance_error"]
        std_dev = statistics["std_dev_fk"]
        distances_train.append(distance_error)
        variances_train.append(std_dev)
        for i, data_folder_test in enumerate(data_folders_test):
            statistics = compute_statistics(model_step, data_folder_test, offset_distance=offset_distance)
            distance_error = statistics["distance_error"]
            std_dev = statistics["std_dev_fk"]
            distances_test[i].append(distance_error)
            variances_test[i].append(std_dev)
    
    percentage_improved_distance_train= (distances_train[0] - distances_train[-1])/distances_train[0] * 100
    percentage_improved_variance_train= (variances_train[0] - variances_train[-1])/variances_train[0] * 100
    percentage_improved_distance_test =  [(distances_test[i][0] - distances_test[i][-1])/distances_test[i][0] * 100 for i in range(len(data_folders_test))]
    percentage_improved_variance_test =  [(variances_test[i][0] - variances_test[i][-1])/variances_test[i][0] * 100 for i in range(len(data_folders_test))]
    print (f"The consistency when from {variances_train[0]:.4E} to {variances_train[-1]:.4E} on the training data")
    print (f"The distortion when from {distances_train[0]:.4E} to {distances_train[-1]:.4E} on the training data")
    print (f"The consistency when from {np.mean(np.array(variances_test)[:,0]):.4E} to {np.mean(np.array(variances_test)[:,-1]):.4E} on the test data on average")
    print (f"The distortion when from {np.mean(np.array(distances_test)[:,0]):.4E} to {np.mean(np.array(distances_test)[:,-1]):.4E} on the test data on average")
    print(f"Percentage of removed error on training set: {percentage_improved_variance_train}")
    print(f"Percentage of removed distortion error on training set: {percentage_improved_distance_train}")
    print(f"Percentage of removed error on test set: {np.mean(percentage_improved_variance_test)}")
    print(f"Percentage of removed distortion error on test set {np.mean(percentage_improved_distance_test)}")

def plot_training_curves(model_folder: str, data_folder_train: str, data_folders_test: List[str], offset_distance, latex=False) -> None:

    if latex == True:
        plt.rcParams.update({
        "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
        "text.usetex": False,          # Use LaTeX for text rendering
        "font.family": "serif",       # Use a serif font
        "pgf.rcfonts": False,         # Disable using Matplotlib's default font settings
        })
    kpis = yaml.load(open(f"{model_folder}/kpis.yaml"), Loader=yaml.FullLoader)
    nb_steps = kpis["nb_steps"]
    train_name = data_folder_train.split("/")[-1]

    steps = list(range(nb_steps))
    distances_train = []
    distances_test = [[] for _ in data_folders_test]
    variances_train = []
    variances_test = [[] for _ in data_folders_test]
    average_z_train = []
    average_z_test = [[] for _ in data_folders_test]
    print(f"Offset distance: {offset_distance}")

    std_devs_z = []

    for step in steps:
        print(f"Step {step}")
        model_step = URDF.load(f"{model_folder}/step_{step}/model.urdf")
        statistics = compute_statistics(model_step, data_folder_train, offset_distance=offset_distance)
        distance_error = statistics["distance_error"]
        std_dev = statistics["std_dev_fk"]
        average_z = statistics["height_error"]
        average_z_train.append(average_z)
        distances_train.append(distance_error)
        variances_train.append(std_dev)
        average_z_all = []
        for i, data_folder_test in enumerate(data_folders_test):
            statistics = compute_statistics(model_step, data_folder_test, offset_distance=offset_distance)
            distance_error = statistics["distance_error"]
            std_dev = statistics["std_dev_fk"]
            average_z = statistics["height_error"]
            distances_test[i].append(distance_error)
            variances_test[i].append(std_dev)
            average_z_all.append(average_z)
        std_dev_z = np.std(np.array(average_z_all))
        std_devs_z.append(std_dev_z)



    fig, ax = plt.subplots(1, 1)
    # make log scale
    ax.set_yscale("log")
    ax.plot(steps, distances_train, label="train", color='blue')
    for i, distances in enumerate(distances_test):
        name = data_folders_test[i].split("/")[-1]
        #ax.plot(steps, distances, label=f"{name.replace('_', '-')}", alpha=0.5)
        if i == 0:
            ax.plot(steps, distances, label="validation", color='orange', alpha=0.5)
        else:
            ax.plot(steps, distances, color='orange', alpha=0.5)

    # set legend
    fontsize=16
    ax.legend(fontsize=fontsize)
    ax.set_xlabel("Step", fontsize=fontsize)
    ax.set_ylabel("$\epsilon$ [m]", fontsize=fontsize)
    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    # only show the step at every 5th
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))

    ax.set_ylim(1e-7, 1e-2)
    # set title

    # the figure should have tight margins but the legend should not be cut off
    # make size of figure suitable for journal paper IEEE standard
    # fig.set_size_inches(3.487/2, 1.5)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    # plt.subplots_adjust(left=0.35, right=1.0, top=1.0, bottom=0.38)

    if latex:
        plt.savefig(f"{model_folder}/distortion.pgf")
    else:
        plt.savefig(f"{model_folder}/distortion.png")

    fig, ax = plt.subplots(1, 1)

    ax.set_yscale("log")
    #ax[1].plot(steps, variances_train, label=train_name.replace('_', '-'), color='black')
    ax.plot(steps, variances_train, label="train", color='blue')
    for i, variances in enumerate(variances_test):
        name = data_folders_test[i].split("/")[-1]
        #ax.plot(steps, variances, label=f"{name.replace('_', '-')}", alpha=0.5)
        if i == 0:
            ax.plot(steps, variances, label="validation", color='orange', alpha=0.5)
        else:
            ax.plot(steps, variances, color='orange', alpha=0.5)

    ax.legend(fontsize=fontsize)
    # can you put the x-axis label at the right end below the figure?
    ax.set_xlabel("Step", fontsize=fontsize)
    ax.set_ylabel("$\sigma$ [m]",fontsize=fontsize)

    ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    # only show the step at every 5th
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.set_ylim(1e-4, 3e-2)

    # make size of figure suitable for journal paper IEEE standard
    # fig.set_size_inches(3.487/2, 1.5)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


    # plt.subplots_adjust(left=0.35, right=1.0, top=1.0, bottom=0.38)
    if latex:
        plt.savefig(f"{model_folder}/consistency.pgf")
    else:
        plt.savefig(f"{model_folder}/consistency.png")
    """

    ax[2].set_yscale("log")
    ax[2].plot(steps, std_devs_z, color='blue')
    
    #ax[2].set_xlabel("Step")
    ax[2].xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax[2].set_ylim(1e-4, 1e-1)
    ax[2].set_title("Height consistency")
    """
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
