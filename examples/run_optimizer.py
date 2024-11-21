import sys
import numpy as np
import casadi as ca
from copy import deepcopy
import os
from calibrate_fk.parameter_optimizer import ParameterOptimizer
import pprint


def main():
    optimizer = ParameterOptimizer()
    optimizer.load_model()
    optimizer.read_data()
    optimizer.create_symbolic_fk("panda_link0", "panda_hand_tcp")
    optimizer.select_parameters()
    optimizer.create_fk_expression()
    optimizer.evaluate_fks(verbose=True)
    optimizer.optimize()
    optimizer.evaluate_fks(verbose=True)
    optimizer.modify_urdf_parameters('test.urdf')


if __name__ == "__main__":
    main()
