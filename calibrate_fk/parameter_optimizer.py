import numpy as np
from typing import Dict, Tuple
import casadi as ca
import xml.etree.ElementTree as ET
from forwardkinematics import GenericURDFFk

class ParameterOptimizer():
    _params: Dict[str, Dict[str, ca.SX]]
    _best_params: dict
    _symbolic_fk: GenericURDFFk
    _data: np.ndarray

    def __init__(self, urdf: str, root_link: str, end_link: str, params: Dict[str, Dict[str, ca.SX]]):
        self._symbolic_fk = GenericURDFFk(
            urdf,
            root_link=root_link,
            end_links=[end_link],
        )
        self._urdf = urdf
        self._params = params
        self._best_params = {}
        for joint_name, joint_params in self._params.items():
            self._best_params[joint_name] = {}
            for param_name, param in joint_params.items():
                self._best_params[joint_name][param_name] = 0
        self._q = ca.SX.sym("q", self._symbolic_fk.n())
        self._fk_casadi_expr = self._symbolic_fk.casadi(self._q, end_link, position_only=True, symbolic_parameters=self._params)
        self._fk_casadi_expr_pure = self._symbolic_fk.casadi(self._q, end_link, position_only=True)
        self._fk_fun_pure = ca.Function("fk_pure", [self._q], [self._fk_casadi_expr_pure])

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

    def read_data(self, file_path: str):
        self._data = np.loadtxt(file_path, delimiter=",")

    def optimize(self):
        fks = []
        for joint_angles in self._data:
            substituted_fk = ca.substitute(self._fk_casadi_expr, self._q, joint_angles)
            fks.append(substituted_fk)

        fk_mean = ca.sum2(ca.horzcat(*fks)) / len(fks)
        fk_variance = ca.sum2(ca.horzcat(*[(fk - fk_mean)**2 for fk in fks])) / len(fks)
        fk_variance_norm = ca.sum1(fk_variance)

        pairwise_comparison_objective = 0
        for i in range(len(fks)):
            for j in range(i, len(fks)):
                if i == j:
                    continue
                diff = ca.norm_2(fks[i] - fks[j])**2
                symbols = ca.symvar(diff)
                pairwise_comparison_objective += diff







        #objective = q[0]**2
        parameter_list = self.list_parameters()
        #problem = {'x': parameter_list, 'f': pairwise_comparison_objective}
        # Add constraints
        problem = {'x': parameter_list, 'f': fk_variance_norm}
        # set learning rate /step size
        solver_options = {'ipopt': {
            'print_level': 0,
            }
        }
        solver = ca.nlpsol('solver', 'ipopt', problem, solver_options)
        x0 = self.list_best_parameters()
        parameter_space_width = 0.01
        lbx = x0 - parameter_space_width
        ubx = x0 + parameter_space_width
        solution = solver(x0=x0, lbx=lbx, ubx=ubx)
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


    def evaluate_fks(self) -> Tuple[np.ndarray, np.ndarray]:
        fks = np.zeros((len(self._data), 3))
        fk_exp = ca.substitute(self._fk_casadi_expr, self.list_parameters(), self.list_best_parameters())
        fk_fun = ca.Function('fk_eval', [self._q], [fk_exp])
        for i, joint_angles in enumerate(self._data):
            fks[i] = np.array(fk_fun(joint_angles)).flatten()
            #print(self._fk_fun_pure(joint_angles))
            #print(fks[i])

        fk_mean = np.mean(fks, axis=0)
        fk_variance = np.var(fks, axis=0)
        return fk_mean, fk_variance


    def modify_urdf_parameters(self, urdf_file: str, output_file: str):
        """
        Modify the URDF file's joint parameters (xyz, rpy) based on the param_dict
        and save the modified URDF to a new file.

        Args:
        - urdf_file (str): Path to the input URDF file.
        - param_dict (dict): Dictionary containing joint names as keys and their corresponding
                             'x', 'y', 'z', 'roll', 'pitch', 'yaw' values as sub-keys.
        - output_file (str): Path to the output URDF file.
        """
        # Load URDF file
        tree = ET.parse(urdf_file)
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



