import os
import numpy as np
from typing import Dict, Tuple, Optional, List, Set
import casadi as ca
import xml.etree.ElementTree as ET
from forwardkinematics import GenericURDFFk
import tkinter as tk
from tkinter import filedialog

root = tk.Tk()
root.withdraw()

class UrdfNotLoadedException(Exception):
    pass

class ParameterOptimizer():
    _params: Dict[str, Dict[str, ca.SX]]
    _best_params: dict
    _symbolic_fk: GenericURDFFk
    _data_1: np.ndarray
    _data_2: np.ndarray
    _offset_distance: float = 0.05
    _urdf: str
    _urdf_file: str
    _end_link: str

    def __init__(self):
        pass

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

    def select_parameters(self) -> None:
        selected_parameters = []
        for joint in self._symbolic_fk.robot.active_joints():
            joint_is_active = input(f"Should the joint {joint} be optimized? (y/n): ")
            if joint_is_active.lower() == "y":
                selected_parameters.append(joint)
        self.create_parameters(selected_parameters)

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

    def create_parameters(self, selected_joints: List[str]) -> None:
        self._params = {}
        self._best_params = {}
        for joint in selected_joints:
            self._params[joint] = {
                "x": ca.SX.sym(f"{joint}_x"),
                "y": ca.SX.sym(f"{joint}_y"),
                "z": ca.SX.sym(f"{joint}_z"),
                "roll": ca.SX.sym(f"{joint}_roll"),
                "pitch": ca.SX.sym(f"{joint}_pitch"),
                "yaw": ca.SX.sym(f"{joint}_yaw"),
            }
            self._best_params[joint] = {
                "x": np.random.uniform(-0.1, 0.1),
                "y": np.random.uniform(-0.1, 0.1),
                "z": np.random.uniform(-0.1, 0.1),
                "roll": np.random.uniform(-0.1, 0.1),
                "pitch": np.random.uniform(-0.1, 0.1),
                "yaw": np.random.uniform(-0.1, 0.1),
            }

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

    def read_data(self, folder: Optional[str] = None) -> None:
        if not folder:
            default_data_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../ros_ws")
            print("Select the folder containing the recorded data.")
            recording_folder = filedialog.askdirectory(
                title="Select a data folder.",
                initialdir=default_data_directory,
            )
        else:
            recording_folder = folder
        if not recording_folder or not os.path.exists(recording_folder):
            raise FileNotFoundError(f"Recording folder {recording_folder} not found.")
        file_path_1 = os.path.join(recording_folder, "hole_0.csv")
        if not os.path.exists(file_path_1):
            raise FileNotFoundError(f"Data file {file_path_1} not found.")
        file_path_2 = os.path.join(recording_folder, "hole_1.csv")
        if not os.path.exists(file_path_2):
            raise FileNotFoundError(f"Data file {file_path_2} not found.")
        self._data_1 = np.loadtxt(file_path_1, delimiter=",")
        self._data_2 = np.loadtxt(file_path_2, delimiter=",")

    def optimize(self):
        self._fk_fun_pure = ca.Function("fk_pure", [self._q], [self._fk_casadi_expr_pure])
        fks_1 = []
        for joint_angles_1 in self._data_1:
            substituted_fk = ca.substitute(self._fk_casadi_expr, self._q, joint_angles_1)
            fks_1.append(substituted_fk)
        fks_2 = []
        for joint_angles_2 in self._data_2:
            substituted_fk = ca.substitute(self._fk_casadi_expr, self._q, joint_angles_2)
            fks_2.append(substituted_fk)

        fk_mean_1 = ca.sum2(ca.horzcat(*fks_1)) / len(fks_1)
        fk_variance_1 = ca.sum2(ca.horzcat(*[(fk - fk_mean_1)**2 for fk in fks_1])) / len(fks_1)
        fk_variance_norm_1 = ca.sum1(fk_variance_1)
        fk_mean_2 = ca.sum2(ca.horzcat(*fks_2)) / len(fks_2)
        fk_variance_2 = ca.sum2(ca.horzcat(*[(fk - fk_mean_2)**2 for fk in fks_2])) / len(fks_2)
        fk_variance_norm_2 = ca.sum1(fk_variance_2)

        distance_error = (ca.norm_2(fk_mean_1[0:2] - fk_mean_2[0:2]) - self._offset_distance)**2
        height_error = ca.norm_2(fk_mean_1[2] - fk_mean_2[2])**2 


        objective = fk_variance_norm_1 + fk_variance_norm_2 + distance_error + height_error


        parameter_list = self.list_parameters()
        # Add constraints
        problem = {'x': parameter_list, 'f': objective}
        # set learning rate /step size
        solver_options = {'ipopt': {
            'print_level': 0,
            }
        }
        solver = ca.nlpsol('solver', 'ipopt', problem, solver_options)
        x0 = self.list_best_parameters()
        parameter_space_width = 1.01
        lbx = x0 - parameter_space_width
        ubx = x0 + parameter_space_width
        solution = solver(x0=x0)#, lbx=lbx, ubx=ubx)
        solution_list = np.array(solution['x'])[:, 0].tolist()
        for i in range(len(solution_list)):
            symbol = parameter_list[i]
            value = solution_list[i]
            # split only at the last underscore
            joint_name, param_name = symbol.name().rsplit("_", 1)
            self._best_params[joint_name][param_name] = value

    @property
    def best_params(self):
        return self._best_params

    @best_params.setter
    def best_params(self, value):
        self._best_params = value


    def evaluate_fks(self, verbose: bool = False) -> dict:
        fk_exp = ca.substitute(self._fk_casadi_expr, self.list_parameters(), self.list_best_parameters())
        fk_fun = ca.Function('fk_eval', [self._q], [fk_exp])

        fks_1 = np.zeros((len(self._data_1), 3))
        fks_2 = np.zeros((len(self._data_2), 3))
        for i, joint_angles in enumerate(self._data_1):
            fks_1[i] = np.array(fk_fun(joint_angles)).flatten()
        for i, joint_angles in enumerate(self._data_2):
            fks_2[i] = np.array(fk_fun(joint_angles)).flatten()

        fk_mean_1 = np.round(np.mean(fks_1, axis=0), decimals=4)
        fk_variance_1 = np.round(np.var(fks_1, axis=0), decimals=4)
        fk_mean_2 = np.round(np.mean(fks_2, axis=0), decimals=4)
        fk_variance_2 = np.round(np.var(fks_2, axis=0), decimals=4)
        distance_error = np.round(np.linalg.norm(fk_mean_1 - fk_mean_2) - self._offset_distance, decimals=4)
        if verbose:
            print(f"Mean_1: {fk_mean_1}")
            print(f"Variance_1: {fk_variance_1}")
            print(f"Mean_2: {fk_mean_2}")
            print(f"Variance_2: {fk_variance_2}")
            print(f"Distance Error: {distance_error}")
            print(f"Height Error: {fk_mean_1[2] - fk_mean_2[2]}")
        kpis = {
            "mean_1": fk_mean_1,
            "var_1": fk_variance_1,
            "mean_2": fk_mean_2,
            "var_2": fk_variance_2,
            "distance": distance_error,
        }
        return kpis



    def modify_urdf_parameters(self, output_file: str):
        """
        Modify the URDF file's joint parameters (xyz, rpy) based on the param_dict
        and save the modified URDF to a new file.

        Args:
        - param_dict (dict): Dictionary containing joint names as keys and their corresponding
                             'x', 'y', 'z', 'roll', 'pitch', 'yaw' values as sub-keys.
        - output_file (str): Path to the output URDF file.
        """
        # Load URDF file
        tree = ET.parse(self._urdf_file)
        root = tree.getroot()

        # Iterate through all 'joint' elements
        for joint in root.findall('joint'):
            joint_name = joint.get('name')

            # If the joint exists in the dictionary, update its origin values
            if joint_name in self._best_params:
                origin = joint.find('origin')
                if origin is not None:
                    xyz_values = f"{self._best_params[joint_name]['x']} {self._best_params[joint_name]['y']} {self._best_params[joint_name]['z']}"
                    rpy_values = f"{self._best_params[joint_name]['roll']} {self._best_params[joint_name]['pitch']} {self._best_params[joint_name]['yaw']}"
                    
                    origin.set('xyz', xyz_values)
                    origin.set('rpy', rpy_values)

        # Save the modified URDF to the output file
        tree.write(output_file, xml_declaration=True, encoding='utf-8')
        print(f"Modified URDF saved to {output_file}")



