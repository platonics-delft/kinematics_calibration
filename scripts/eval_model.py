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
argument_parser.add_argument("--model", "-m", help="calibrated robot model. Saved in the calibrated_urdf folder. ")
argument_parser.add_argument("--data", "-d", help="Specify the folder to evaluate the model on. The data should be stored in the data folder.")
argument_parser.add_argument("--overlay", help="Overlay the images", action="store_true")
argument_parser.add_argument("--without-mesh", "-wm", help="Use meshes(default) or replace with cylinders", action="store_true") # Add a flag to enable/disable the cylinder replacement
argument_parser.add_argument("--camera-settings", "-c", help="Camera settings file", default="camera_settings.json")
argument_parser.add_argument("--offset-distance", "-od", help="Offset distance for the hole", default=0.05, type=float)


args = argument_parser.parse_args()

model = args.model
with_mesh = not args.without_mesh
data = args.data
# show_urdf = True
overlay = args.overlay
script_directory = os.path.abspath(__file__)
parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
output_folder = parent_directory + "/calibrated_urdf/" + model
camera_setting_file = args.camera_settings
offset_distance = float(args.offset_distance)
overwrite = True

model_config = yaml.load(open(output_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name = model_config["urdf"]

urdf_file = output_folder + "/" + urdf_name + ".urdf"
eval_folder = os.path.abspath(os.path.join(parent_directory, 'data', data))

## extract the last part of the path 
data_folder_name = data.split('/')[-1]

os.makedirs(f"{output_folder}/images", exist_ok=True)


robot = yourdfpy.URDF.load(urdf_file)
dof = len(robot.actuated_joints)
q_show = [0, ] * dof
counter = 0
for joint in robot.robot.joints:
    if joint.type == "revolute":
        q_show[counter] = (joint.limit.lower + joint.limit.upper) / 2
        counter += 1


if eval_folder:
    q_hole_0, q_hole_1 = read_data(eval_folder)
    images = []
    q_0 = q_hole_0[0]
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



