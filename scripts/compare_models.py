import numpy as np
import casadi as ca
from copy import deepcopy
import os
from calibrate_fk.model_comparison import ModelComparison


def main():
    absolute_path = os.path.dirname(os.path.abspath(__file__))
    URDF_FILE_1=absolute_path + "/../assets/panda.urdf"
    URDF_FILE_2=absolute_path + "/../assets/panda_calibrated_grounded.urdf"
    DATA_FILE_1=absolute_path + "/../data/angles_panda_validate_1.csv"
    DATA_FILE_2=absolute_path + "/../data/angles_panda_validate_2.csv"
    with open(URDF_FILE_1, "r") as file:
        urdf_1 = file.read()
    with open(URDF_FILE_2, "r") as file:
        urdf_2 = file.read()

    comparison = ModelComparison(urdf_1, urdf_2, 'panda_link0', 'panda_hand_tcp')

    comparison.read_data(DATA_FILE_1, DATA_FILE_2)
    comparison.compare_models()
if __name__ == "__main__":
    main()
