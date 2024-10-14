# Calibration of robot kinematics

Have you ever wondered why the accuracy of your robot is poor, and why the
end-effector is at different poses when you move in the null-space? Well, 
the platonics did! And we are you to help you out. This problem is especally 
relevant when working with cartesian impedance control.

The method implemented in this repository was mainly developed for the Franka
Emika Panda robot, where inaccurate kinematics have been raised a few times.

The technique and code used here can also be used to determine mount positions
of robots on different tables or other robots.

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

## Quick Start

The calibration is done in two steps. First, the robot is moved to a few
poses, and the joint states are recorded. This is done by running the
`record_joint_states_dataset` script.

```bash
rosrun calibrate_tools record_joint_states_dataset
```
Place the end-effector ball inside the hollow sphere and press `<SPACE>`.
Repeat the step with different joint configuration around 20 times.
End the script by pressing `<CTRL+C>`.
The script will write a csv file with the
joint states.
The joint states are stored in a csv file called `joint_angles.csv`. Move
the file to the `data` folder in the root of the repository. 
Repeat the step with different joint configuration for the second hollow sphere. 
Move the resulting file in the `data` folder as well but under a different name.

The second step is the optimization. Make sure you have activated your virtual
environment if you use one (`source bin/activate`, `poetry shell`). Go to the
example folder and run  the optimization script.

```bash
cd examples
python3 run_optimizer.py <path_to_first_csv> <path_to_second_csv>
```

This script runs the optimazation and prints the result. It outputs a new `urdf`
file for the panda robot with the optimized parameters called `test.urdf`.


