import os
import tkinter as tk
import xml.etree.ElementTree as ET
from copy import deepcopy
from tkinter import filedialog
from typing import Dict, List, Optional, Set, Tuple
from pprint import pprint

import casadi as ca
import numpy as np
import yaml
import yourdfpy
from forwardkinematics import GenericURDFFk

from calibrate_fk.utils import evaluate_model, read_data, replace_mesh_with_cylinder

root = tk.Tk()
root.withdraw()

class UrdfNotLoadedException(Exception):
    pass

class IterationStepCallback(ca.Callback):
    def __init__(self, name, nx, opts={}):
        ca.Callback.__init__(self)
        self.nx = nx
        self.solutions = []  # List to store intermediate solutions
        self.construct(name, opts)


    def get_n_in(self): 
        return ca.nlpsol_n_out()  # Number of inputs expected by the solver

    def get_n_out(self): 
        return 1  # Number of outputs from the callback

    def get_name_in(self, i): 
        return ca.nlpsol_out(i)  # Names of inputs

    def get_name_out(self, i): 
        return "ret"  # Single output

    def get_sparsity_in(self, i):
        name = ca.nlpsol_out(i)
        if name == "x":  # Decision variable
            return ca.Sparsity.dense(self.nx)
        elif name in ("f"):  # Cost
            return ca.Sparsity.scalar()
        elif name == "g":  # Constraints (empty in this case)
            return ca.Sparsity(0, 0)
        else:
            return ca.Sparsity(0, 0)


    def eval(self, arg):
        x_solution = arg[ca.nlpsol_out().index("x")]
        self.solutions.append([float(v) for v in np.array(x_solution)[:, 0].tolist()])
        return [0]



class ParameterOptimizer():
    _params: Dict[str, Dict[str, ca.SX]]
    _best_params: dict
    _initial_params: dict
    _residuals: dict
    _symbolic_fk: GenericURDFFk
    _data_folder: str
    _data_1: np.ndarray
    _data_2: np.ndarray
    _offset_distance: float = 0.05
    _regulizer_weight: float = 1e-4

    _urdf: str
    _urdf_file: str
    _end_link: str
    _output_folder : str
    _nb_steps: int = 0

    def __init__(self, output_folder: str):
        self._output_folder = output_folder

    def set_offset_distance(self, offset_distance: float):
        self._offset_distance = offset_distance

    def set_regulizer_weight(self, regulizer_weight: float):
        self._regulizer_weight = regulizer_weight

    def load_model(self, filename: Optional[str] = None) -> None:
        if filename is None:
            default_model_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../assets")
            print("Select a urdf model.")
            self._urdf_file = filedialog.askopenfilename(
                title="Select a urdf model",
                initialdir=default_model_directory,
            )
        else: 
            self._urdf_file = filename
            self._model = yourdfpy.URDF.load(self._urdf_file)

        if not self._urdf_file or not os.path.exists(self._urdf_file):
            raise FileNotFoundError(f"URDF file {self._urdf_file} not found.")
        with open(self._urdf_file, "r") as file:
            self._urdf = file.read()

    def create_symbolic_fk(self, root_link: str, end_link: str) -> None:
        if not self._urdf:
            raise UrdfNotLoadedException("No urdf specified, load urdf first.")
        self._end_link = end_link
        self._symbolic_fk = GenericURDFFk(
            self._urdf,
            root_link=root_link,
            end_links=[self._end_link],
        )
        self._q = ca.SX.sym("q", self._symbolic_fk.n())

    def select_parameters(self, variance_noise: float = 0.0, selected_parameters: Optional[List[str]] = None) -> None:
        if selected_parameters is None:
            selected_parameters = []
            for joint in self._symbolic_fk.robot.active_joints():
                joint_is_active = input(f"Should the joint {joint} be optimized? (y/n): ")
                if joint_is_active.lower() == "y":
                    selected_parameters.append(joint)
        self.create_parameters(selected_parameters, variance_noise=variance_noise)

    @property
    def active_joints(self) -> Set[str]:
        return self._symbolic_fk.robot.active_joints()

    @property
    def available_links(self) -> Set[str]:
        tree = ET.parse(self._urdf_file)
        root = tree.getroot()

        links = set()

        # Iterate through all elements and find <link> tags
        for link in root.iter('link'):
            links.add(link.attrib['name'])

        return links

    def create_parameters(self, selected_joints: List[str], variance_noise: float = 0.0) -> None:
        self._params = {}
        self._best_params = {}
        self._initial_params = {}
        self._residuals = {}
        for joint in selected_joints:
            self._params[joint] = {
                "x": ca.SX.sym(f"{joint}_x"),
                "y": ca.SX.sym(f"{joint}_y"),
                "z": ca.SX.sym(f"{joint}_z"),
                "roll": ca.SX.sym(f"{joint}_roll"),
                "pitch": ca.SX.sym(f"{joint}_pitch"),
                "yaw": ca.SX.sym(f"{joint}_yaw"),
            }
            # use as initial guess the values from the URDF
            values = self.get_original_parameters_for_joint(joint)
            self._best_params[joint] = {
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "roll": values[3],
                "pitch": values[4],
                "yaw": values[5],
            }
            self._initial_params[joint] = {
                "x": values[0] + np.random.normal(0, variance_noise),
                "y": values[1] + np.random.normal(0, variance_noise),
                "z": values[2] + np.random.normal(0, variance_noise),
                "roll": values[3] + np.random.normal(0, variance_noise),
                "pitch": values[4] + np.random.normal(0, variance_noise),
                "yaw": values[5] + np.random.normal(0, variance_noise),
            }

            self._residuals[joint] = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }

    def get_original_parameters_for_joint(self, joint_name: str) -> List[float]:
        tree = ET.parse(self._urdf_file)
        root = tree.getroot()

        # Iterate through all 'joint' elements
        for joint in root.findall('joint'):
            if joint.get('name') == joint_name:
                origin = joint.find('origin')
                if origin is not None:
                    xyz_values = origin.get('xyz')
                    rpy_values = origin.get('rpy')
                    return [float(val) for val in xyz_values.split()] + [float(val) for val in rpy_values.split()]
        raise ValueError(f"Joint {joint_name} not found in URDF file.")

    def create_fk_expression(self) -> None:
        self._fk_casadi_expr = self._symbolic_fk.casadi(self._q, self._end_link, position_only=True, symbolic_parameters=self._params)
        self._fk_casadi_expr_pure = self._symbolic_fk.casadi(self._q, self._end_link, position_only=True)


    def list_parameters(self) -> ca.SX:
        parameter_list = []
        for joint_name, joint_params in self._params.items():
            for _, param in joint_params.items():
                parameter_list.append(param)
        return ca.vertcat(*parameter_list)

    def list_best_parameters(self) -> np.ndarray:
        parameter_list = []
        for joint_name, joint_params in self._best_params.items():
            for _, param in joint_params.items():
                parameter_list.append(param)
        return np.array(parameter_list)

    def read_data(self, folder: str, number_samples: Optional[int] = None) -> None:
        self._data_folder = folder
        # create an empty dictionary to store the data
        self.data = {} 
        # for each of the subfolders in the folder, read the data
        self.data[folder] = read_data(folder=os.path.join(folder, folder))
        if number_samples is not None:
            for i in range(len(self.data[folder])):
                self.data[folder][i] = self.data[folder][i][np.random.choice(self.data[folder][i].shape[0], number_samples, replace=False)]
    def read_multiple_data(self, folder: str, number_samples: Optional[int] = None) -> None:
        self._data_folder = folder
        # create an empty dictionary to store the data
        self.data = {} 
        # for each of the subfolders in the folder, read the data
        for subfolder in os.listdir(folder):
            self.data[subfolder] = read_data(folder=os.path.join(folder, subfolder))
            if number_samples is not None:
                for i in range(len(self.data[subfolder])):
                    self.data[subfolder][i] = self.data[subfolder][i][np.random.choice(self.data[subfolder][i].shape[0], number_samples, replace=False)]

    def optimize(self, saving_steps: bool = False):
        self.create_fk_expression()
        self._fk_fun_pure = ca.Function("fk_pure", [self._q], [self._fk_casadi_expr_pure])
        # for each element in the dictionary 
        objective = 0
        for sockets in self.data.values():
            fks_1 = []
            for joint_angles_1 in sockets[0]:
                substituted_fk = ca.substitute(self._fk_casadi_expr, self._q, joint_angles_1)
                fks_1.append(substituted_fk)
            fks_2 = []
            for joint_angles_2 in sockets[1]:
                substituted_fk = ca.substitute(self._fk_casadi_expr, self._q, joint_angles_2)
                fks_2.append(substituted_fk)

            fk_mean_1 = ca.sum2(ca.horzcat(*fks_1)) / len(fks_1)
            N1 = len(fks_1)
            fk_variance_1 = ca.sum2(ca.horzcat(*[(fk - fk_mean_1)**2 for fk in fks_1])) / len(fks_1)
            fk_variance_norm_1 = ca.sum1(fk_variance_1)
            fk_mean_2 = ca.sum2(ca.horzcat(*fks_2)) / len(fks_2)
            N2 = len(fks_2)
            fk_variance_2 = ca.sum2(ca.horzcat(*[(fk - fk_mean_2)**2 for fk in fks_2])) / len(fks_2)
            fk_variance_norm_2 = ca.sum1(fk_variance_2)

            distance_error = (ca.norm_2(fk_mean_1 - fk_mean_2) - self._offset_distance)
            distance_error_squared = distance_error**2
            objective += (fk_variance_norm_1* N1 + fk_variance_norm_2 *N2) /(N1 + N2) + distance_error_squared

        residuals = []
        for joint_name, joint_params in self._params.items():
            for param_name, param in joint_params.items():
                param_epsilon = param - self._initial_params[joint_name][param_name]
                #aggregate onlyy if the name of the joint is not ball_joint
                if joint_name != "ball_joint":
                    residuals.append(param_epsilon**2)
        objective += self._regulizer_weight * ca.sum1(ca.vertcat(*residuals))

        parameter_list = self.list_parameters()
        # Add constraints
        problem = {'x': parameter_list, 'f': objective}
        nx = parameter_list.size()[0]
        # set learning rate /step size
        calibration_iteration_callback = IterationStepCallback("iteration_callback", nx=nx)
        solver_options = {
            'ipopt': {
                'print_level': 0,
            },

        }
        if saving_steps:
            solver_options['iteration_callback'] = calibration_iteration_callback
        solver = ca.nlpsol('solver', 'ipopt', problem, solver_options)
        x0 = self.list_best_parameters()
        solution = solver(x0=x0)#, lbx=lbx, ubx=ubx)
        self._nb_steps = solver.stats()['iter_count']
        solution_list = np.array(solution['x'])[:, 0].tolist()
        for i in range(len(solution_list)):
            symbol = parameter_list[i]
            value = solution_list[i]
            # split only at the last underscore
            joint_name, param_name = symbol.name().rsplit("_", 1)
            self._best_params[joint_name][param_name] = value
        self.urdf_name = os.path.basename(self._urdf_file)
        output_file = os.path.join(self._output_folder,self.urdf_name)
        self.modify_urdf_parameters(output_file, self._best_params)
        self._model = yourdfpy.URDF.load(output_file)

        for joint_name, joint_params in self._best_params.items():
            for param_name, param in joint_params.items():
                param_residual = param - self._initial_params[joint_name][param_name]
                self._residuals[joint_name][param_name] = param_residual
        if saving_steps:
            for i, intermediate_solution in enumerate(calibration_iteration_callback.solutions):
                self.save_intermediate_solution(intermediate_solution, parameter_list, i)


    def save_intermediate_solution(self, intermediate_solution: List[float], parameter_list: List[ca.SX], i: int):
            intermediate_parameters = deepcopy(self._best_params)
            for j in range(len(intermediate_solution)):
                symbol = parameter_list[j]
                value = intermediate_solution[j]
                joint_name, param_name = symbol.name().rsplit("_", 1)
                intermediate_parameters[joint_name][param_name] = value
            intermediate_folder = f"{self._output_folder}/step_{i}"
            os.makedirs(intermediate_folder, exist_ok=True)
            intermediate_urdf = f"{intermediate_folder}/model.urdf"
            self.modify_urdf_parameters(intermediate_urdf, intermediate_parameters)
            replace_mesh_with_cylinder(intermediate_urdf, intermediate_urdf)
            intermediate_model = yourdfpy.URDF.load(intermediate_urdf)
            kpis = evaluate_model(intermediate_model, self._data_folder, verbose=False)
            with open(f"{intermediate_folder}/kpis.yaml", 'w') as f:
                yaml.dump(kpis, f)



            

    @property
    def best_params(self):
        return self._best_params

    @best_params.setter
    def best_params(self, value):
        self._best_params = value


    def evaluate_fks(self, verbose: bool = False) -> dict:
        kpis = evaluate_model(self._model, self._data_folder, verbose=verbose)
        kpis['nb_steps'] = self._nb_steps
        with open(f"{self._output_folder}/kpis.yaml", 'w') as f:
            yaml.dump(kpis, f)
        return kpis



    def modify_urdf_parameters(self, output_file: str, parameters: dict):
        """
        Modify the URDF file's joint parameters (xyz, rpy) based on the param_dict
        and save the modified URDF to a new file.

        Args:
        - output_file (str): Path to the output URDF file.
        - param_dict (dict): Dictionary containing joint names as keys and their corresponding
                             'x', 'y', 'z', 'roll', 'pitch', 'yaw' values as sub-keys.
        """
        # Load URDF file
        tree = ET.parse(self._urdf_file)
        root = tree.getroot()

        # Iterate through all 'joint' elements
        for joint in root.findall('joint'):
            joint_name = joint.get('name')

            # If the joint exists in the dictionary, update its origin values
            if joint_name in parameters:
                origin = joint.find('origin')
                if origin is not None:
                    xyz_values = f"{parameters[joint_name]['x']} {parameters[joint_name]['y']} {parameters[joint_name]['z']}"
                    rpy_values = f"{parameters[joint_name]['roll']} {parameters[joint_name]['pitch']} {parameters[joint_name]['yaw']}"
                    
                    origin.set('xyz', xyz_values)
                    origin.set('rpy', rpy_values)

        # Save the modified URDF to the output file
        tree.write(output_file, xml_declaration=True, encoding='utf-8')
        print(f"Modified URDF saved to {output_file}")



