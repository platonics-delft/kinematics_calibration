# Calibration of robot kinematics

Have you ever wondered why the accuracy of your robot is poor, and why the
end-effector is at different poses when you move in the null-space? Well, 
the platonics did! And we are you to help you out. This problem is especally 
relevant when working with cartesian control where an accurate forward kinematics is desirable. 

The method implemented in this repository was mainly developed for the Franka
Emika Panda robot, where inaccurate kinematics have been raised a few times.

The technique and code used here can also be used to determine mount positions
of robots on different tables or other robots.

## The calibration tool

<!-- TODO: Add picture and description of the calibration tool -->

## Installation

Install the package through pip, using 
```bash
pip3 install .
```

Install the package through poetry, using
```bash
poetry install
```

Install the ros workspace to record data. It is only running one script to 
echo joint states and writing a csv file. There is hardly any dependencies
on this.

```bash
cd ros_ws
catkin build
source devel/setup.{bash,zsh}
```

## Record the data

The calibration is done in two steps. First, the robot is moved to a few
poses, and the joint states are recorded. This is done by running the
`record_joint_states_dataset` script.

Here is an example for calibrating the kuka for example: 

```bash
rosrun calibration_tools record_joint_states_dataset --joint-state-topic-name /joint_states --robot-name kuka_1  --tool-position-on-table front --robot-dof 7 
```
- Press 'a' to add a new joint configuration that has the sphere in the hole tha the algorithm 
- Presss 'd' to delete the last joint configuration from the list
- Press 's' when you switch the hole in which you are recording
- Press 'q' when you have finished recording

### Record data with a Franka 
```bash
rosrun calibration_tools record_joint_states_panda --joint-state-topic-name /joint_states --robot-name panda_1 --config-file panda_1.yaml --tool-position-on-table front 
```

- Press 'check' to add a data point
- Press 'down' to delete the last data point
- Press 'o' to switch between holes
- Press 'x' to save the data and quit.
You need to create a config file for the 
/src/calibration_tools/config/

## Calibrate the urdf model
The second step is the optimization. Make sure you have activated your virtual
environment if you use one (`source bin/activate`, `poetry shell`). Go to the
**scripts folder** and run  the optimization script.

```bash
cd scripts
python3 run_optimizer.py --urdf <path/to/urdf> --data <path/to/data/folder> 
```

To get the full list of arguments, run the following command:

```bash
python3 run_optimizer.py --help
```


This script runs the optimazation and prints the result. It outputs a new `urdf`
in the asset folder with the optimized parameters. KPI's are stored in the
output folder (default is `output`).

## Evaluate a calibration

You can evaluate the calibration by runnig.

```bash
cd examples
python3 eval_model.py --urdf-file <path/to/urdf> --evaluate-on <path/to/data/folder> 
```

This scripts produces the kpis.
By passing the `-s` flag, you can show the robot.
By passing the `--overlay` flag an overlay of all the configuration is saved to
the output folder.

To get all the potential arguments, run the following command:

```bash
python3 eval_model.py --help
```
