import argparse
import os
from calibrate_fk.utils import modify_urdf

def main():
    parser = argparse.ArgumentParser(description='Create plots for the calibration data')
    parser.add_argument('--model', "-m", type=str, help='Calibrated robot model. Saved in the calibrated_urdf folder, e.g. panda_1')
    args = parser.parse_args()

    model = args.model
    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    folder_file = parent_directory + "/calibrated_urdf/" + model 
    urdf_file = folder_file+ "/panda.urdf"


    # Define the pattern to search for and the replacement string
    search_pattern = r'\.\./meshes/panda/'
    replace_with = 'package://franka_description/meshes/visual/'

    # Path to save the new URDF file
    output_file = folder_file+ '/panda_calibrated.urdf'

    # Call the function to modify the URDF
    modify_urdf(urdf_file, search_pattern, replace_with, output_file)

if __name__ == "__main__":
    main()







