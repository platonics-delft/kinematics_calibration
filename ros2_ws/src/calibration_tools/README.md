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

## Features

- **Selective Joint Recording**: Specify exactly which joints to record by name
- **Joint Name Preservation**: Joint names are saved as headers in CSV files for clear data organization
- **Interactive Recording**: Real-time keyboard controls for data collection
- **Backward Compatibility**: Supports both old and new CSV file formats
- **Multi-Robot Support**: Works with different robot types by specifying appropriate joint names

## Usage Examples

### Franka Panda Robot
```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name panda_1 \
  --tool-position-on-table center \
  --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```

### KUKA iiwa14 Robot
```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name kuka_14 \
  --tool-position-on-table left \
  --joint-names iiwa_joint_1 iiwa_joint_2 iiwa_joint_3 iiwa_joint_4 iiwa_joint_5 iiwa_joint_6 iiwa_joint_7
```

### Kinova Gen3 Lite Robot
```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name kinova_gen3 \
  --tool-position-on-table center \
    --joint-names joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
```

## General Usage

### Method 1: Using the ROS2 executable (recommended)

```bash
ros2 run calibration_tools record_joint_states_dataset \
  --joint-state-topic-name /joint_states \
  --robot-name panda_1 \
  --tool-position-on-table center \
  --joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```

## Parameters

- `joint_state_topic_name`: Topic where joint states are published (default: `/joint_states`)
- `robot_name`: Name of the robot being calibrated (e.g., `panda_1`, `kuka_1`)
- `tool_position_on_table`: Position of the tool on the table
- `joint_names`: List of specific joint names to record (space-separated, required)

## Key Controls

When the recorder is running:
- Press `a` to add a data point
- Press `d` to delete the last data point
- Press `s` to switch between holes
- Press `i` to show help information
- Press `q` to save data and quit

## Data Storage

Joint state data is saved in CSV format in the `data/{robot_name}/{tool_position}/` directory relative to the workspace root. The CSV files now include:

- **Joint names as headers**: Each CSV file contains the joint names as a header row (prefixed with #)
- **Organized data**: Joint positions are saved in the same order as the specified joint names
- **Backward compatibility**: The system can read both old CSV files (without headers) and new ones (with headers)

### CSV File Format

```csv
#panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7
0.123,-0.456,0.789,-0.234,0.567,-0.890,0.345
0.234,-0.567,0.890,-0.345,0.678,-0.901,0.456
```

## Changes from ROS1 to ROS2

1. **Node Initialization**: Changed from `rospy.init_node()` to ROS2 Node class inheritance
2. **Publishers/Subscribers**: Updated to use ROS2 syntax with QoS profiles
3. **Timing**: Replaced `rospy.Rate` with ROS2 timers
4. **Package Discovery**: Updated from `rospkg` to `ament_index_python`
5. **Build System**: Migrated from catkin to ament_cmake
6. **Package Structure**: Added proper Python package structure with `setup.py`
7. **Launch Files**: Created ROS2 Python-based launch files

## Recent Updates

### Version 2.0 - Joint Name Support
- **Selective Joint Recording**: Now supports specifying exact joint names to record instead of just DOF count
- **Enhanced Data Format**: CSV files include joint names as headers for better data organization
- **Improved Joint Filtering**: The recorder filters incoming joint state messages to only record specified joints
- **Backward Compatibility**: Maintains compatibility with existing CSV files without headers

### Migration from DOF-based to Name-based Recording

**Old Usage (deprecated):**
```bash
--robot-dof 7
```

**New Usage (recommended):**
```bash
--joint-names panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7
```

## Troubleshooting

If you encounter import errors, make sure to:

1. Build the package with `colcon build`
2. Source the workspace with `source install/setup.bash`
3. Ensure all dependencies are installed

For any issues with package discovery, the fallback path resolution should work, but you may need to adjust the relative paths based on your workspace structure.