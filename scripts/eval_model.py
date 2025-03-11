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

from calibrate_fk.utils import evaluate_model, read_data, check_data_path
argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--model", "-m", help="calibrated robot model. Saved in the calibrated_urdf folder. ")
argument_parser.add_argument("--data", "-t", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")
argument_parser.add_argument("--offset-distance", "-od", help="Offset distance between the two sockets", default=0.05, type=float)


args = argument_parser.parse_args()

model = args.model
data = args.data
script_directory = os.path.abspath(__file__)
parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
output_folder = parent_directory + "/calibrated_urdf/" + model
offset_distance = float(args.offset_distance)
overwrite = True

model_config = yaml.load(open(output_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name = model_config["urdf"]

urdf_file = output_folder + "/" + urdf_name + ".urdf"

eval_folder = os.path.abspath(os.path.join(parent_directory, 'data', data[0]))
print(f"evaluating on the data relative to the tool position in  {eval_folder}")

data_path = [None] * len(data)
for i, d in enumerate(data):
    data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))

check_data_path(data_path)

robot = yourdfpy.URDF.load(urdf_file)

if data_path:
    print(f"evaluating on the data relative to the tool position in  {data_path}")
    statistics = evaluate_model(robot, data_path, offset_distance=offset_distance, verbose=True)
else:
    print("No data provided")
    sys.exit(1)
