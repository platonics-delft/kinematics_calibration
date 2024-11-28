import os
import xml.etree.ElementTree as ET
from tkinter import filedialog
import yaml
from typing import Optional, Tuple, List

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from yourdfpy import URDF

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
    width, height = images[0].shape[1], images[0].shape[0]
    assert all(img.shape[0] == height and img.shape[1] == width for img in images), "All images must have the same dimensions!"

    average_image = sum(images) / len(images)
    average_image = np.clip(average_image, 0, 255).astype(np.uint8)
    result_image = Image.fromarray(average_image, mode="RGBA")

    result_image.save(output_path)

def evaluate_model(robot_model: URDF, data_folder: str, verbose: bool = False) -> dict:

    data_hole_0, data_hole_1 = read_data(data_folder)


    fks_1 = np.zeros((len(data_hole_0), 3))
    fks_2 = np.zeros((len(data_hole_1), 3))
    for i, joint_angles in enumerate(data_hole_0):
        robot_model.update_cfg(joint_angles)
        fk = robot_model.get_transform(frame_from=robot_model.base_link, frame_to="ball_link")
        fks_1[i] = np.array(fk[:3, 3]).flatten()
    for i, joint_angles in enumerate(data_hole_1):
        robot_model.update_cfg(joint_angles)
        fk = robot_model.get_transform(frame_from=robot_model.base_link, frame_to="ball_link")
        fks_2[i] = np.array(fk[:3, 3]).flatten()

    fk_mean_1 = np.round(np.mean(fks_1, axis=0), decimals=10)
    fk_variance_1 = np.round(np.var(fks_1, axis=0), decimals=10)
    fk_mean_2 = np.round(np.mean(fks_2, axis=0), decimals=10)
    fk_variance_2 = np.round(np.var(fks_2, axis=0), decimals=10)
    distance_error = np.round(np.linalg.norm(fk_mean_1 - fk_mean_2) - OFFSET_DISTANCE, decimals=10)
    if verbose:
        print(f"Mean_1: {fk_mean_1}")
        print(f"Variance_1: {fk_variance_1}")
        print(f"Mean_2: {fk_mean_2}")
        print(f"Variance_2: {fk_variance_2}")
        print(f"Distance Error: {distance_error}")
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

def compute_statistics(model: URDF, data_folder: str) -> tuple:
    print(f"Data folder {data_folder}")
    data_hole_0, data_hole_1 = read_data(data_folder)

    fks_1 = np.zeros((len(data_hole_0), 3))
    fks_2 = np.zeros((len(data_hole_1), 3))
    for i, joint_angles in enumerate(data_hole_0):
        model.update_cfg(joint_angles)
        fk = model.get_transform(frame_from=model.base_link, frame_to="ball_link")
        fks_1[i] = np.array(fk[:3, 3]).flatten()
    for i, joint_angles in enumerate(data_hole_1):
        model.update_cfg(joint_angles)
        fk = model.get_transform(frame_from=model.base_link, frame_to="ball_link")
        fks_2[i] = np.array(fk[:3, 3]).flatten()


    fk_mean_1 = np.mean(fks_1, axis=0)
    fk_mean_2 = np.mean(fks_2, axis=0)
    fk_variance_1 = np.sum(np.var(fks_1, axis=0))
    fk_variance_2 = np.sum(np.var(fks_2, axis=0))
    distance_error = np.abs(np.linalg.norm(fk_mean_1 - fk_mean_2) - OFFSET_DISTANCE)
    std_dev = np.sqrt(fk_variance_1 + fk_variance_2)
    return float(distance_error), std_dev



def plot_distance_curves(model_folder: str, data_folder_train: str, data_folders_test: List[str]) -> None:

    steps = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    kpis = yaml.load(open(f"{model_folder}/kpis.yaml"), Loader=yaml.FullLoader)
    nb_steps = kpis["nb_steps"]

    steps = list(range(nb_steps))
    distances_train = []
    distances_test = [[] for _ in data_folders_test]
    variances_train = []
    variances_test = [[] for _ in data_folders_test]

    for step in steps:
        model_step = URDF.load(f"{model_folder}/step_{step}/model.urdf")
        distance_error, std_dev = compute_statistics(model_step, data_folder_train)
        distances_train.append(distance_error)
        variances_train.append(std_dev)
        for i, data_folder_test in enumerate(data_folders_test):
            distance_error, std_dev = compute_statistics(model_step, data_folder_test)
            distances_test[i].append(distance_error)
            variances_test[i].append(std_dev)
    # 2 plots
    fig, ax = plt.subplots(1, 2)
    # make log scale
    ax[0].set_yscale("log")
    ax[0].plot(steps, distances_train, label="Train", color='orange')
    for i, distances in enumerate(distances_test):
        ax[0].plot(steps, distances, label=f"Test {i}", color='blue', alpha=0.5)
    # set legend
    ax[0].legend()
    ax[0].set_xlabel("Step")

    ax[1].set_yscale("log")
    ax[1].plot(steps, variances_train, label="Train", color='orange')
    for i, variances in enumerate(variances_test):
        ax[1].plot(steps, variances, label=f"Test {i}", color='blue', alpha=0.5)

    ax[1].legend()
    ax[1].set_xlabel("Step")


    plt.show()





