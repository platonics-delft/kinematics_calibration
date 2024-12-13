# MUKCa: Minimal User-friendly Kinematics Calibration of Robot Manipulators

Have you ever wondered why the accuracy of your robot is poor, and why the
end-effector is at different poses when you move in the null-space? Well, 
the platonics did! And we are you to help you out. This problem is especally 
relevant when working with cartesian control where an accurate forward kinematics is desirable. 

The method implemented in this repository was mainly developed for the Franka
Emika Panda robot, where inaccurate kinematics have been raised a few times but was tested on many other commercial robots. This is the first tool that does not require any external systems like optitrack, camera sensors or laser trackers. You will only need one simple tool to place in front of the robot. 
The tool is 3D printable and it is composed of two spherical sockets placed at a distance of 50 mm. A 3D printable spherical joint is then attached to the end effector and used to record many position of the robt in each of the sockets, identified as 0 and 1. 
![alt text](imgs/mukca_tool.png)
The first step is the data recording and then the optimization is performed to optimize the robot urdf file. 

## Installation
This is a python package named calibrated_fk. 
Install the package through pip, using 
```bash
pip3 install -e .
```

Or install the package through poetry, using
```bash
poetry install
```
## Record the data
![Alt Text](imgs/mucka.gif)

To record the data, we rely on the ros topic that has the joint angles of the encoders. Place this repository in a catkin workspace and build it. 

```bash
cd ros_ws
catkin build
source devel/setup.{bash,zsh}
```
This will install the ros workspace to record data. It is only running one script to 
save joint states and writing a csv file. There is hardly any dependencies on this.
The calibration routine is very simple. The tool has two sockets identified as 0 and 1. There is a sphere attached at the end effecotor. The idea is to record main joint configuration of the ball in the socket 0 and many other in the socket 1. At least 20 for each socket. To do this we run the node in calibration_tools named record_joint_states_dataset. 



Here is an example for calibrating the kuka for example: 

```bash
rosrun calibration_tools record_joint_states_dataset --joint-state-topic-name /joint_states --robot-name kuka_1  --tool-position-on-table front --robot-dof 7 
```
For the robot name we advice to use the name of the robot and a number considering that you want to calibrate more than one robot with the same name. The position of the tool is to distinguish the different datasets if more one for different calibration tool are recorded.  

What to do when the code runs: 

- Press 'a' to add a new joint configuration that has the sphere in the socket that the displayed in the algorithm.  
- Presss 'd' to delete the last joint configuration from the list
- Press 's' when you switch the hole in which you are recording
- Press 'q' when you have finished recording. 


### Record data with a Franka 
![Alt Text](imgs/mukca_franka.gif)

As controllers, we suggest to use the [Human Friendly Controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers.git). You can also lunch any controller of [franka_ros](https://github.com/frankaemika/franka_ros.git). It is important to check that the joint state can be echoed. 

When using a Franka, we give the possibility of using the buttons on the end effector to add data or switch between holes. 
Run the record_joint_states_panda node, e.g., 
```bash
rosrun calibration_tools record_joint_states_panda --joint-state-topic-name /joint_states --robot-name panda_1 --config-file panda_1.yaml --tool-position-on-table front 
```

- Press 'check' to add a data point
- Press 'down' to delete the last data point
- Press 'o' to switch between holes
- Press 'x' to save the data and quit.

You need to create a config file for the panda in 
'ros_ws/src/calibration_tools/config/<robot-name>' where you are specifying: hostname, username, password,that are the IP for the panda, the username and password used to access the Desk interface. 

## Calibrate the urdf model
We are ready to optimize our model. 

<!-- Make sure you have activated your virtual
environment if you use one (`source bin/activate`, `poetry shell`).  -->

Go to the
**scripts folder** and run  the optimization script.

```bash
cd scripts
```
Then you can run all the scripts as: 
```bash
python3 run_optimizer.py --model <nomial_urdf_name> --data <path/to/data/folder> 
```

The nomial model needs to be in the **urdf** folder. For example panda.urdf. The data are the one saved in the previous step. For example panda_1/front. 
This script runs the optimazation and prints the result. It outputs a new `urdf` model with the same name of the original one, e.g. panda.urdf but located in the **calibrated_urdf** folder and subfolder of the id that we gave to that robot, e.g. panda_1. So now if you browse to calibrated_urdf/panda_1 you will find your panda.urdf that contains calibrated parameters. 
```bash
python3 run_optimizer.py --model panda --data panda_1/front
``` 

## Plot the learning curves
To diplay how good the model gets, we diplay the learning and the validation curves by running 

```bash
python3 create_plots.py --model panda_1
```
This will display the consistency and the distortion of the model both on the training set but also in any other of the dataset that is in the folder data/panda_1. 

## Evaluate a calibrated model on a particular dataset

You can evaluate the calibrated model on any dataset that you recorded for that robot

```bash
python3 eval_model.py --model <calibrated_model> --data <path/to/data/folder> 
```
For example, 

```bash
python3 eval_model.py  --model panda_1 --data panda_1/front
```

By passing the `--overlay` flag an overlay of all the configuration is saved to the calibrated_urdf/panda_1.

If the optimization was successful the overlay will look like this: 


![Alt Text](imgs/overlay.PNG)

You can notice that the forward kinematics is very consistent by the diplayed red dot of the learned shpere attached at the end effector. 