import numpy as np
from typing import Dict
import casadi as ca
from forwardkinematics import GenericURDFFk
import matplotlib.pyplot as plt

class ModelComparison():
    _symbolic_fk: GenericURDFFk
    _data_1: np.ndarray
    _data_2: np.ndarray
    _urdf_1: str
    _urdf_2: str
    _offset_distance: float = 0.05

    def __init__(self, urdf_1: str, urdf_2: str, root_link: str, end_link: str):
        self._symbolic_fk_1 = GenericURDFFk(
            urdf_1,
            root_link=root_link,
            end_links=[end_link],
        )
        self._symbolic_fk_2 = GenericURDFFk(
            urdf_2,
            root_link=root_link,
            end_links=[end_link],
        )
        self._end_link = end_link
        self._urdf_1 = urdf_1
        self._urdf_2 = urdf_2


    def read_data(self, file_path_1: str, file_path_2: str):
        self._data_1 = np.loadtxt(file_path_1, delimiter=",")
        self._data_2 = np.loadtxt(file_path_2, delimiter=",")


    def compare_models(self, verbose: bool = False) -> tuple:
        fks_model_1_data_1 = np.zeros((len(self._data_1), 3))
        fks_model_2_data_1 = np.zeros((len(self._data_1), 3))
        fks_model_1_data_2 = np.zeros((len(self._data_2), 3))
        fks_model_2_data_2 = np.zeros((len(self._data_2), 3))
        for i, joint_angles in enumerate(self._data_1):
            fks_model_1_data_1[i] = self._symbolic_fk_1.numpy(joint_angles, child_link=self._end_link, position_only=True)
            fks_model_2_data_1[i] = self._symbolic_fk_2.numpy(joint_angles, child_link=self._end_link, position_only=True)
        for i, joint_angles in enumerate(self._data_2):
            fks_model_1_data_2[i] = self._symbolic_fk_1.numpy(joint_angles, child_link=self._end_link, position_only=True)
            fks_model_2_data_2[i] = self._symbolic_fk_2.numpy(joint_angles, child_link=self._end_link, position_only=True)

        # 3d scatter plot with different colors for each of the four fks
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # set axis ranges to min and max of all data
        ax.set_xlim([min(np.min(fks_model_1_data_1[:, 0]), np.min(fks_model_1_data_2[:, 0]), np.min(fks_model_2_data_1[:, 0]), np.min(fks_model_2_data_2[:, 0])), 
             max(np.max(fks_model_1_data_1[:, 0]), np.max(fks_model_1_data_2[:, 0]), np.max(fks_model_2_data_1[:, 0]), np.max(fks_model_2_data_2[:, 0]))])

        ax.set_ylim([min(np.min(fks_model_1_data_1[:, 1]), np.min(fks_model_1_data_2[:, 1]), np.min(fks_model_2_data_1[:, 1]), np.min(fks_model_2_data_2[:, 1])), 
                     max(np.max(fks_model_1_data_1[:, 1]), np.max(fks_model_1_data_2[:, 1]), np.max(fks_model_2_data_1[:, 1]), np.max(fks_model_2_data_2[:, 1]))])

        ax.set_zlim([min(np.min(fks_model_1_data_1[:, 2]), np.min(fks_model_1_data_2[:, 2]), np.min(fks_model_2_data_1[:, 2]), np.min(fks_model_2_data_2[:, 2])), 
                     max(np.max(fks_model_1_data_1[:, 2]), np.max(fks_model_1_data_2[:, 2]), np.max(fks_model_2_data_1[:, 2]), np.max(fks_model_2_data_2[:, 2]))])


        ax.scatter(fks_model_1_data_1[:, 0], fks_model_1_data_1[:, 1], fks_model_1_data_1[:, 2], c='red', label='Model 1 Data 1')
        ax.scatter(fks_model_1_data_2[:, 0], fks_model_1_data_2[:, 1], fks_model_1_data_2[:, 2], c='red', label='Model 1 Data 2')
        ax.scatter(fks_model_2_data_1[:, 0], fks_model_2_data_1[:, 1], fks_model_2_data_1[:, 2], c='blue', label='Model 2 Data 1')
        ax.scatter(fks_model_2_data_2[:, 0], fks_model_2_data_2[:, 1], fks_model_2_data_2[:, 2], c='blue', label='Model 2 Data 2')
        plt.show()

        mean_model_1_data_1 = np.mean(fks_model_1_data_1, axis=0)
        mean_model_2_data_1 = np.mean(fks_model_2_data_1, axis=0)
        mean_model_1_data_2 = np.mean(fks_model_1_data_2, axis=0)
        mean_model_2_data_2 = np.mean(fks_model_2_data_2, axis=0)

        distance_model_1 = mean_model_1_data_1 - mean_model_1_data_2
        distance_model_2 = mean_model_2_data_1 - mean_model_2_data_2

        print(f"Distance model 1: {distance_model_1}")
        print(f"Distance model 2: {distance_model_2}")

        # divide by 0.025 m
        discrete_distance_model_1 = distance_model_1 / 0.025
        discrete_distance_model_2 = distance_model_2 / 0.025

        print(f"Discrete distance model 1: {discrete_distance_model_1}")
        print(f"Discrete distance model 2: {discrete_distance_model_2}")


        

