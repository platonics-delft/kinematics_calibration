# Calibration Tools ROS2 Package

This package provides tools for kinematic calibration, specifically for recording joint states data.

## Package Structure

This package has been converted from ROS1 to ROS2 and includes:

- **Joint State Recorder**: A Python node that records joint states for calibration purposes
- **Launch files**: ROS2 launch files for easy execution
- **Backward compatibility**: The original scripts are preserved for compatibility

## Dependencies

- ROS2 (tested with Humble/Iron)
- Python packages:
  - numpy
  - pynput
  - rclpy
  - sensor_msgs

## Installation

1. Build the package:
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select calibration_tools
```

2. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Method 1: Using the ROS2 executable (recommended)

```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name panda_1 \
  --tool-position-on-table center \
  --robot-dof 7
```

### Method 2: Using the launch file

```bash
ros2 launch calibration_tools record_joint_states.launch.py \
  joint_state_topic_name:=/joint_states \
  robot_name:=panda_1 \
  tool_position_on_table:=center \
  robot_dof:=7
```

### Method 3: Using the legacy scripts (backward compatibility)

The original scripts are still available in the `lib/calibration_tools/` directory after installation.

## Parameters

- `joint_state_topic_name`: Topic where joint states are published (default: `/joint_states`)
- `robot_name`: Name of the robot being calibrated (e.g., `panda_1`, `kuka_1`)
- `tool_position_on_table`: Position of the tool on the table
- `robot_dof`: Degrees of freedom of the robot (default: 7)

## Key Controls

When the recorder is running:
- Press `a` to add a data point
- Press `d` to delete the last data point
- Press `s` to switch between holes
- Press `i` to show help information
- Press `q` to save data and quit

## Data Storage

Joint state data is saved in CSV format in the `data/{robot_name}/{tool_position}/` directory relative to the workspace root.

## Changes from ROS1 to ROS2

1. **Node Initialization**: Changed from `rospy.init_node()` to ROS2 Node class inheritance
2. **Publishers/Subscribers**: Updated to use ROS2 syntax with QoS profiles
3. **Timing**: Replaced `rospy.Rate` with ROS2 timers
4. **Package Discovery**: Updated from `rospkg` to `ament_index_python`
5. **Build System**: Migrated from catkin to ament_cmake
6. **Package Structure**: Added proper Python package structure with `setup.py`
7. **Launch Files**: Created ROS2 Python-based launch files

## Troubleshooting

If you encounter import errors, make sure to:
1. Build the package with `colcon build`
2. Source the workspace with `source install/setup.bash`
3. Ensure all dependencies are installed

For any issues with package discovery, the fallback path resolution should work, but you may need to adjust the relative paths based on your workspace structure.