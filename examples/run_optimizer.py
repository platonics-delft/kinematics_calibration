import sys
import numpy as np
import casadi as ca
from copy import deepcopy
import os
from calibrate_fk.parameter_optimizer import ParameterOptimizer
import pprint


def main():
    absolute_path = os.path.dirname(os.path.abspath(__file__))
    URDF_FILE=absolute_path + "/../assets/panda.urdf"
    DATA_FILE_1=absolute_path + '/' + sys.argv[1]
    DATA_FILE_2=absolute_path + '/' + sys.argv[2]
    symbolic_params = {
        "panda_joint1": {
            "x": ca.SX.sym("panda_joint1_x"),
            "y": ca.SX.sym("panda_joint1_y"),
            "z": ca.SX.sym("panda_joint1_z"),
            "roll": ca.SX.sym("panda_joint1_roll"),
            "pitch": ca.SX.sym("panda_joint1_pitch"),
            "yaw": ca.SX.sym("panda_joint1_yaw"),
        },
        "panda_joint2": {
            "x": ca.SX.sym("panda_joint2_x"),
            "y": ca.SX.sym("panda_joint2_y"),
            "z": ca.SX.sym("panda_joint2_z"),
            "roll": ca.SX.sym("panda_joint2_roll"),
            "pitch": ca.SX.sym("panda_joint2_pitch"),
            "yaw": ca.SX.sym("panda_joint2_yaw"),
        },
        "panda_joint3": {
            "x": ca.SX.sym("panda_joint3_x"),
            "y": ca.SX.sym("panda_joint3_y"),
            "z": ca.SX.sym("panda_joint3_z"),
            "roll": ca.SX.sym("panda_joint3_roll"),
            "pitch": ca.SX.sym("panda_joint3_pitch"),
            "yaw": ca.SX.sym("panda_joint3_yaw"),
        },
        "panda_joint4": {
            "x": ca.SX.sym("panda_joint4_x"),
            "y": ca.SX.sym("panda_joint4_y"),
            "z": ca.SX.sym("panda_joint4_z"),
            "roll": ca.SX.sym("panda_joint4_roll"),
            "pitch": ca.SX.sym("panda_joint4_pitch"),
            "yaw": ca.SX.sym("panda_joint4_yaw"),
        },
        "panda_joint5": {
            "x": ca.SX.sym("panda_joint5_x"),
            "y": ca.SX.sym("panda_joint5_y"),
            "z": ca.SX.sym("panda_joint5_z"),
            "roll": ca.SX.sym("panda_joint5_roll"),
            "pitch": ca.SX.sym("panda_joint5_pitch"),
            "yaw": ca.SX.sym("panda_joint5_yaw"),
        },
        "panda_joint6": {
            "x": ca.SX.sym("panda_joint6_x"),
            "y": ca.SX.sym("panda_joint6_y"),
            "z": ca.SX.sym("panda_joint6_z"),
            "roll": ca.SX.sym("panda_joint6_roll"),
            "pitch": ca.SX.sym("panda_joint6_pitch"),
            "yaw": ca.SX.sym("panda_joint6_yaw"),
        },
        "panda_joint7": {
            "x": ca.SX.sym("panda_joint7_x"),
            "y": ca.SX.sym("panda_joint7_y"),
            "z": ca.SX.sym("panda_joint7_z"),
            "roll": ca.SX.sym("panda_joint7_roll"),
            "pitch": ca.SX.sym("panda_joint7_pitch"),
            "yaw": ca.SX.sym("panda_joint7_yaw"),
        },
        "panda_tcp_ball_joint": {
            'z': ca.SX.sym("panda_tcp_ball_joint_z"),
            'x': ca.SX.sym("panda_tcp_ball_joint_x"),
            'y': ca.SX.sym("panda_tcp_ball_joint_y"),
            'roll': ca.SX.sym("panda_tcp_ball_joint_roll"),
            'pitch': ca.SX.sym("panda_tcp_ball_joint_pitch"),
            'yaw': ca.SX.sym("panda_tcp_ball_joint_yaw"),
        },
    }






    guessed_params = {
        "panda_joint1": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.333,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint2": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": -1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint3": {
            "x": 0.0,
            "y": -0.316,
            "z": 0.0,
            "roll": 1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint4": {
            "x": 0.0825,
            "y": 0.0,
            "z": 0.0,
            "roll": 1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint5": {
            "x": -0.0825,
            "y": 0.384,
            "z": 0.0,
            "roll": -1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint6": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_joint7": {
            "x": 0.088,
            "y": 0.0,
            "z": 0.0,
            "roll": 1.5707963267948966,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "panda_tcp_ball_joint": {
            'z': 0.03,
            'x': 0.0,
            'y': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        },
    }

    guessed_params_copy = deepcopy(guessed_params)


    #symbolic_params_selected = {"panda_joint6": symbolic_params["panda_joint6"]}
    #guessed_params_selected = {"panda_joint6": guessed_params["panda_joint6"]}

    

    with open(URDF_FILE, "r") as file:
        urdf = file.read()
    optimizer = ParameterOptimizer(urdf,'panda_link0',"panda_ball",symbolic_params)
    optimizer.best_params = guessed_params
    optimizer.read_data(DATA_FILE_1, DATA_FILE_2)
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize()
    optimizer.evaluate_fks(verbose=True)
    #pprint.pprint(optimizer.best_params)
    optimizer.modify_urdf_parameters(URDF_FILE, 'test.urdf')

    #with open(URDF_FILE, "r") as file:
    #    urdf = file.read()
    #optimizer = ParameterOptimizer(urdf,'panda_link0',"panda_hand_tcp",symbolic_params)
    #optimizer.best_params = guessed_params_copy
    #print(optimizer.best_params)
    #optimizer.read_data(DATA_FILE_VALIDATION)
    #fk_mean, fk_var = optimizer.evaluate_fks()
    #print(f"Mean Validation: {fk_mean}")
    #print(f"Variance Validation: {fk_var}")


if __name__ == "__main__":
    main()
