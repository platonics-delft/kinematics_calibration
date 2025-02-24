import argparse
import json
import os
import sys
from io import BytesIO
import matplotlib.pyplot as plt
import numpy as np
import yaml
import yourdfpy
from PIL import Image
import random

from calibrate_fk.utils import (overlay_images,
                                read_data)

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--model", "-m", help="calibrated robot model. Saved in the calibrated_urdf folder. ")
argument_parser.add_argument("--data", "-d", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")
argument_parser.add_argument("--camera-settings", "-c", help="Camera settings file", default="camera_settings.json")
argument_parser.add_argument("--no-generate-images", "-n", help="Pass this flag if you do not want to generate images again but only perform the overlay", action="store_true")
argument_parser.add_argument("--max-images", "-mi", help="Maximum number of images to overlay", default=20, type=int)
args = argument_parser.parse_args()

model = args.model
data = args.data
max_images = args.max_images
script_directory = os.path.abspath(__file__)
parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
model_folder = parent_directory + "/calibrated_urdf/" + model
camera_setting_file = args.camera_settings
generate_images = not(args.no_generate_images)

if not os.path.exists(model_folder):
    print(f"model {model} does not exist in calibrated_urdf. Did you already run the optimizer?")
    # Check if the data folder exists and the urdf path exists, otherwise exit
    data_root = os.path.abspath(os.path.join(parent_directory, 'data'))
    print("Available calibrated models that can be evaluated are: ")
    # List first level directories
    for d in sorted(os.listdir(data_root)):
        if d == "README.md":
            continue
        print(f"    {d}")
    sys.exit(1)


model_config = yaml.load(open(model_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name = model_config["urdf"]

urdf_file = model_folder + "/" + urdf_name + ".urdf"

data_path = [None] * len(data)
for i, d in enumerate(data):
    data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))
for data_path_ith in data_path:
    if not os.path.exists(data_path_ith):
        print(f"Data folder {data_path_ith} does not exist.")
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

robot = yourdfpy.URDF.load(urdf_file)
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



if generate_images:
    # breakpoint()
    data= read_data(data_path)

    # iterated over the keys of data 
    for keys, joints in data.items():
        print(keys)
        # breakpoint()
        ## extract the last part of the path 
        # data_folder_name = data.split('/')[-1]

        os.makedirs(f"{model_folder}/images/{keys}", exist_ok=True)

        images = []
        #read the data in the first key in the dictionary

        q_hole_0 = joints[0]
        q_hole_1 = joints[1]
        q_0 = q_hole_0[0]
        q_show = np.zeros(robot.num_actuated_joints)
        q_show[:len(q_0)]=q_0
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


        for j, q_data in enumerate([q_hole_0, q_hole_1]):
            for i, q in enumerate(q_data):
                print(f"Creating image for hole {j+1} of 2 and config {i+1}/{len(q_data)}")
                q_show[:len(q)]=q
                robot.update_cfg(q_show)
                img_bin = robot.scene.save_image((800, 800))
                img = Image.open(BytesIO(img_bin))

                img_array = np.array(img.convert("RGBA")).astype(np.float32)


                images.append(img_array)
                with open(f'{model_folder}/images/{keys}/hole_{j}_config_{i}.png', 'wb') as f:
                    f.write(img_bin)
        
        # 
    # Open the input image and read the image data in the folder input_path and append it to the input_image list
    output_image = []
    input_path = f"{model_folder}/images/{keys}/"
    output_path = f"{model_folder}/overlay_"+ keys + ".png"
    all_files = os.listdir(input_path)
    image_files = [f for f in all_files if f.endswith(('.png', '.jpg', '.jpeg'))]

    # Select 14 random images
    if len(image_files) >= max_images:
        selected_images = random.sample(image_files, max_images)
    else:
        selected_images = image_files  # If less than 14 images, use all available
    for filename in selected_images:
        with open(input_path + filename, 'rb') as f:
            input_image = f.read()

            # Remove the background using rembg
            output_image.append(input_image)

    overlay_images(output_image, output_path)







# if overlay and not data_path:
#     robot.update_cfg(q_show)
#     robot.show()
#     saved_camera = {
#         'transform': robot.scene.camera_transform.tolist(), 
#         'fov': robot.scene.camera.fov.tolist(),
#     }

#     print(f"Camera settings saved in: {camera_setting_file}")


