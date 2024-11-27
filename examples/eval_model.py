import argparse
import json
import os
import shutil
import sys
from io import BytesIO

import matplotlib.pyplot as plt
import numpy as np
import yaml
import yourdfpy
from PIL import Image

from calibrate_fk.utils import (evaluate_model, overlay_images,
                                plot_fks_iterations, read_data,
                                replace_mesh_with_cylinder)

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--urdf-file", "-u", help="Path to the URDF file")
argument_parser.add_argument("--with-mesh", "-m", help="Use meshes or replace with cylinders(default)", action="store_true") # Add a flag to enable/disable the cylinder replacement
argument_parser.add_argument("--evalauate-on", "-e", help="Specify the folder to evaluate the model on")
argument_parser.add_argument("--show", "-s", help="Show the URDF file", action="store_true")
argument_parser.add_argument("--overlay", help="Overlay the images", action="store_true")
argument_parser.add_argument("--output-folder", "-o", help="Output folder for the results", default="output")
argument_parser.add_argument("--camera-settings", "-c", help="Camera settings file", default="camera_settings.json")
argument_parser.add_argument("--overwrite", "-w", help="Overwrite the output folder", action="store_true")
argument_parser.add_argument("--intermediate", "-i", help="Show intermediate steps", action="store_true")


args = argument_parser.parse_args()

urdf_file = args.urdf_file
with_mesh = args.with_mesh
eval_folder = args.evalauate_on
show_urdf = args.show
overlay = args.overlay
output_folder = "evaluations/" + args.output_folder
camera_setting_file = args.camera_settings
overwrite = args.overwrite
intermediate = args.intermediate
input_folder = urdf_file.split("/")[-1][:-5]

assert os.path.exists(urdf_file), f"URDF file {urdf_file} not found."
assert os.path.exists(input_folder), f"Input folder {input_folder} not found."

# Delete the the output folder if using the default name
if output_folder == "output" or overwrite:
    shutil.rmtree(output_folder, ignore_errors=True)

if os.path.exists(output_folder):

    print(f"Output folder {output_folder} already exists. Please delete it or specify a different folder.")
    sys.exit(1)

os.makedirs(f"{output_folder}/images", exist_ok=True)



if overlay  and eval_folder is None:
    print("Please specify the evaluation folder if you want an overlay.")
    sys.exit(1)

if with_mesh:
    modified_urdf = urdf_file
else:
    modified_urdf = replace_mesh_with_cylinder(urdf_file)

robot = yourdfpy.URDF.load(modified_urdf)
dof = len(robot.actuated_joints)
q_show = [0, ] * dof
counter = 0
for joint in robot.robot.joints:
    if joint.type == "revolute":
        q_show[counter] = (joint.limit.lower + joint.limit.upper) / 2
        counter += 1


if os.path.exists(camera_setting_file):
    with open(camera_setting_file, 'r') as f:
        saved_camera = json.load(f)

    robot.scene.camera_transform = saved_camera['transform']
    robot.scene.camera.fov = saved_camera['fov']



if eval_folder:
    q_hole_0, q_hole_1 = read_data(eval_folder)
    images = []
    q_show = q_hole_0[0]
    if show_urdf:
        robot.update_cfg(q_show)
        print("Move the view such that you can nicely see the end-effector.")
        robot.show()
        saved_camera = {
            'transform': robot.scene.camera_transform.tolist(), 
            'fov': robot.scene.camera.fov.tolist(),
        }

        print(f"Camera settings saved in: {camera_setting_file}")
        with open(camera_setting_file, 'w') as f:
            json.dump(saved_camera, f)

    kpis = evaluate_model(robot, eval_folder, verbose=True)
    with open(f"{output_folder}/kpis.yaml", 'w') as f:
        yaml.dump(kpis, f)


    if overlay:
        for j, q_data in enumerate([q_hole_0, q_hole_1]):
            for i, q in enumerate(q_data):
                print(f"Creating image for hole {j}/2 and config {i}/{len(q_data)}")
                robot.update_cfg(q)
                img_bin = robot.scene.save_image((800, 800))
                img = Image.open(BytesIO(img_bin))

                img_array = np.array(img.convert("RGBA")).astype(np.float32)


                images.append(img_array)
                with open(f'{output_folder}/images/hole_{j}_config_{i}.png', 'wb') as f:
                    f.write(img_bin)
        overlay_images(images, f"{output_folder}/overlay.png")






if show_urdf and not eval_folder:
    robot.update_cfg(q_show)
    robot.show()
    saved_camera = {
        'transform': robot.scene.camera_transform.tolist(), 
        'fov': robot.scene.camera.fov.tolist(),
    }

    print(f"Camera settings saved in: {camera_setting_file}")

if intermediate:
    ax = None
    all_fks = []
    for step in [1, 20, 90]:
        step_folder = f"{input_folder}/step_{step}"
        assert os.path.exists(step_folder), f"Step folder {step_folder} not found."
        kpis_step_file = f"{step_folder}/kpis.yaml"
        kpis = yaml.load(open(kpis_step_file, 'r'), Loader=yaml.FullLoader)
        fks_1 = kpis['fks_1'][0:5]
        fks_2 = kpis['fks_2'][0:5]
        fks = fks_1 #+ fks_2
        all_fks.append(fks)

    ax = plot_fks_iterations(all_fks)
    plt.show()

