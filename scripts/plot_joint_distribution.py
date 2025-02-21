import argparse
import os
import sys
from calibrate_fk.utils import read_data
import numpy as np
import matplotlib.pyplot as plt
def main():

    argument_parser = argparse.ArgumentParser(description='Run the parameter optimizer')
    argument_parser.add_argument("--data", "-t", help="Data that you want to train the calibration on. Stored in data folder.")


    args = argument_parser.parse_args()
    data = args.data
    robot_name = os.path.dirname(data)

    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)

    data_path = os.path.abspath(os.path.join(parent_directory, 'data', data))

    data_1, data_2= read_data(data_path)
    data= np.vstack((data_1, data_2))
    
    #create a subfigure of 7 figures 
    plt.figure()
    for i in range(7):
        plt.subplot(7, 1, i+1)
        plt.hist(data[:,i], bins=20, density=True, alpha=0.6, color='g', label='x')
    plt.show()


if __name__ == "__main__":
    main()
