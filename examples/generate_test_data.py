import numpy as np
import casadi as ca
import os
from scipy.optimize import minimize
from forwardkinematics import GenericURDFFk
import matplotlib.pyplot as plt
from calibrate_fk.plot_planar_3link_arm import plot_robot_arm, forward_kinematics_plot

absolute_path = os.path.dirname(os.path.abspath(__file__))
URDF_FILE=absolute_path + "/../assets/nlink_7.urdf"

with open(URDF_FILE, "r") as file:
    urdf = file.read()
fk_casadi = GenericURDFFk(
    urdf,
    root_link = 'link0',
    end_links=["link7"],
)

q = ca.SX.sym("q", fk_casadi.n())
fk_casadi_expr = fk_casadi.casadi(q, 'link7', position_only=True)

def forward_kinematics(joint_angles):
    """Compute the end-effector position for the robot."""
    fk_value = fk_casadi.numpy(joint_angles, 'link7', position_only=True)
    return fk_value


def error_function(joint_angles, target_pos):
    """Error function to minimize."""
    current_pos = forward_kinematics(joint_angles)
    error = np.linalg.norm(current_pos - target_pos)
    return error

def numerical_inverse_kinematics_casadi(target_pos):
    objective = (fk_casadi_expr[0] - target_pos[0])**2 + (fk_casadi_expr[1] - target_pos[1])**2 + (fk_casadi_expr[2] - target_pos[2])**2
    #objective = q[0]**2
    problem = {'x': q, 'f': objective}
    solver_options = {'ipopt': {'print_level': 0}}
    solver = ca.nlpsol('solver', 'ipopt', problem, solver_options)
    x0 = np.random.rand(fk_casadi.n()) * 2 * np.pi
    solution = solver(x0=x0)
    return np.array(solution['x'])[:, 0].tolist()

# Example usage
random_angle = np.random.rand(fk_casadi.n()) * 2 * np.pi
target_pos = forward_kinematics(random_angle)

try:
    data_set = []
    while len(data_set) < 100:
        print(len(data_set))
        joint_angles = numerical_inverse_kinematics_casadi(target_pos)
        fk = forward_kinematics(joint_angles)
        print(fk)
        if np.linalg.norm(fk - target_pos) > 1e-3:
            continue
        print("fk:", forward_kinematics(joint_angles))
        #plot_robot_arm(joint_angles, link_lengths, len(data_set))

        data_set.append(joint_angles)

    # Save dataset as a CSV file
    np.savetxt("joint_angles.csv", data_set, delimiter=",")


except ValueError as e:
    print(e)


