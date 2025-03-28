import argparse
import os
import yaml
from calibrate_fk.utils import check_data_path, evaluate_model
import yourdfpy
import numpy as np
from scipy.spatial.transform import Rotation as R

def compute_rotation_matrix(x, d):
    # Normalize the vectors
    x = x / np.linalg.norm(x)
    d = d / np.linalg.norm(d)
    
    # Compute the rotation using scipy
    rotation, _ = R.align_vectors(d, x)  # Align x to d
    return rotation.as_matrix()

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--robot_left", "-m", help="calibrated robot model. Saved in the calibrated_urdf folder. ")
argument_parser.add_argument("--robot_right", "-t", help="Data that you want to train the calibration on. Stored in data folder.")
argument_parser.add_argument("--data_left", "-dl", help="Offset distance between the two sockets")
argument_parser.add_argument("--data_right", "-dr", help="Offset distance between the two sockets")
argument_parser.add_argument("--offset-distance", "-od", help="Offset distance between the two sockets", default=0.05, type=float)
args = argument_parser.parse_args()

robot_left = args.robot_left
robot_right = args.robot_right
data_left = args.data_left
data_right = args.data_right
script_directory = os.path.abspath(__file__)
parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
model_left_folder = parent_directory + "/calibrated_urdf/" + robot_left
model_right_folder = parent_directory + "/calibrated_urdf/" + robot_right
offset_distance = float(args.offset_distance)

model_config_left = yaml.load(open(model_left_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name_left = model_config_left["urdf"]
urdf_file_left = model_left_folder + "/" + urdf_name_left + ".urdf"

model_config_right = yaml.load(open(model_right_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name_right = model_config_right["urdf"]
urdf_file_right = model_right_folder + "/" + urdf_name_right + ".urdf"

data_folder_left = os.path.abspath(os.path.join(parent_directory, 'data', data_left))
data_path_left = os.path.abspath(os.path.join(parent_directory, 'data', data_left))

data_folder_right = os.path.abspath(os.path.join(parent_directory, 'data', data_right))
data_path_right = os.path.abspath(os.path.join(parent_directory, 'data', data_right))

#file in data_folder_right
# Get the last folder name in data_path_right
tool_position_right = os.path.basename(os.path.normpath(data_path_right))
tool_position_left = os.path.basename(os.path.normpath(data_path_left))
# print(f"Using data from folder: {last_folder_right}")
data_path_left = [data_path_left]
data_path_right = [data_path_right]
check_data_path(data_path_left)
check_data_path(data_path_right)

robot_left = yourdfpy.URDF.load(urdf_file_left)

robot_right = yourdfpy.URDF.load(urdf_file_right)


statistics_left = evaluate_model(robot_left, data_path_left, offset_distance=offset_distance, verbose=True)
statistics_right = evaluate_model(robot_right, data_path_right, offset_distance=offset_distance, verbose=True)
# breakpoint()
X_r_0= statistics_right[tool_position_right]['mean_1']
X_l_0= statistics_left[tool_position_left]['mean_1']
X_r_1= statistics_right[tool_position_right]['mean_2']
X_l_1= statistics_left[tool_position_left]['mean_2']

# convert to np array 
X_r_0 = np.array(X_r_0).reshape(3,1)
X_l_0 = np.array(X_l_0).reshape(3,1)
X_r_1 = np.array(X_r_1).reshape(3,1)
X_l_1 = np.array(X_l_1).reshape(3,1)

zero_vector = np.zeros((3,1))

d_vector = np.array([0, offset_distance,0]).reshape(3,1)

diff_X_r = X_r_1 - X_r_0

diff_X_l = X_l_1 - X_l_0

R_R = compute_rotation_matrix(diff_X_r.reshape(-1), d_vector.reshape(-1))
R_L = compute_rotation_matrix(diff_X_l.reshape(-1), d_vector.reshape(-1))
    
t_L = - R_L.dot(X_l_0)
t_R = - R_R.dot(X_r_0)

# convert the rotation matrix to roll pitch and yaw angles
roll_L, pitch_L, yaw_L = R.from_matrix(R_L).as_euler('xyz')
roll_R, pitch_R, yaw_R = R.from_matrix(R_R).as_euler('xyz')

# convert the rotation matrix in a quaternion 
quaternion_L = R.from_matrix(R_L).as_quat(scalar_first=True)
quaternion_R = R.from_matrix(R_R).as_quat(scalar_first=True)
# save a yaml file with the structure
# left: 
#     position: 
# x:
# y:
# z:
#     orientation:
# r:
# p:
# y:

calibration_data = {
    'left': {
        'position': {
            'x': float(t_L[0][0]),
            'y': float(t_L[1][0]),
            'z': float(t_L[2][0])
        },
        'orientation': {
            'w': float(quaternion_L[0]),
            'x': float(quaternion_L[1]),
            'y': float(quaternion_L[2]),
            'z': float(quaternion_L[3]),
            
        }
    },
    'right': {
        'position': {
            'x': float(t_R[0][0]),
            'y': float(t_R[1][0]),
            'z': float(t_R[2][0])
        },
        'orientation': {
            'w': float(quaternion_R[0]),
            'x': float(quaternion_R[1]),
            'y': float(quaternion_R[2]),
            'z': float(quaternion_R[3]),
            
        }
    }
}

# breakpoint()
# save the calibration data to a yaml file as reative_position.yaml in the parent directory

calibration_file = os.path.join(parent_directory, 'relative_position.yaml')
with open(calibration_file, 'w') as file:
    yaml.dump(calibration_data, file)

# breakpoint()