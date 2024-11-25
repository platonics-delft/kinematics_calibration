import os
import xml.etree.ElementTree as ET
from tkinter import filedialog
from typing import Optional, Tuple

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

def replace_mesh_with_cylinder(urdf_file) -> str:
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
    modified_urdf = urdf_file.replace(".urdf", "_cylinders.urdf")
    tree.write(modified_urdf)
    print(f"Modified URDF saved as: {modified_urdf}")
    return modified_urdf

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
