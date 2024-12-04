import argparse
import json
import os
import shutil
import sys
from pprint import pprint
from io import BytesIO

import numpy as np
import yaml
import yourdfpy
from PIL import Image

from calibrate_fk.utils import (compute_statistics, overlay_images,
                                read_data,
                                replace_mesh_with_cylinder)

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--urdf-file", "-u", help="Path to the URDF file")
argument_parser.add_argument("--without-mesh", "-m", help="Use meshes(default) or replace with cylinders", action="store_true") # Add a flag to enable/disable the cylinder replacement
argument_parser.add_argument("--evalauate-on", "-e", help="Specify the folder to evaluate the model on")
#argument_parser.add_argument("--show", "-s", help="Show the URDF file", action="store_true")
argument_parser.add_argument("--overlay", help="Overlay the images", action="store_true")
argument_parser.add_argument("--output-folder", "-o", help="Output folder for the results", default="output")
argument_parser.add_argument("--camera-settings", "-c", help="Camera settings file", default="camera_settings.json")
argument_parser.add_argument("--overwrite", "-w", help="Overwrite the output folder", action="store_true")
argument_parser.add_argument("--offset-distance", "-d", help="Offset distance for the hole", default=0.05, type=float)


args = argument_parser.parse_args()

urdf_file = args.urdf_file
with_mesh = not args.without_mesh
eval_folder = args.evalauate_on
show_urdf = True
overlay = args.overlay
output_folder = "evaluations/" + args.output_folder
camera_setting_file = args.camera_settings
offset_distance = float(args.offset_distance)
overwrite = args.overwrite

assert os.path.exists(urdf_file), f"URDF file {urdf_file} not found."

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
    kpis = compute_statistics(robot, eval_folder, offset_distance=offset_distance)
    # kpis_core should be the same but without the keys fks_1 and fks_2
    kpis_core = {k: v for k, v in kpis.items() if k not in ['fks_1', 'fks_2']}
    pprint(kpis_core)
    with open(f"{output_folder}/kpis.yaml", 'w') as f:
        # convert all numpy arrays to list
        kpis = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in kpis.items()}
        # convert all numpy floats to python floats
        kpis = {k: float(v) if isinstance(v, np.float64) else v for k, v in kpis.items()}
        yaml.dump(kpis, f)
    if show_urdf:
        robot.update_cfg(q_show)
        print("Move the view such that you can nicely see the end-effector. Press q in the visualization window to save the camera settings.")
        robot.show()
        saved_camera = {
            'transform': robot.scene.camera_transform.tolist(), 
            'fov': robot.scene.camera.fov.tolist(),
        }

        print(f"Camera settings saved in: {camera_setting_file}")
        with open(camera_setting_file, 'w') as f:
            json.dump(saved_camera, f)



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

