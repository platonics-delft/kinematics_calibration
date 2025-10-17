#!/usr/bin/env python3
import sys
import os
import rclpy
import argparse
from ament_index_python.packages import get_package_share_directory
from calibration_tools.joint_state_recorder import JointStatesRecorder

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Parse arguments
    argument_parser = argparse.ArgumentParser(description='Run the joint states recorder')
    argument_parser.add_argument("--joint-state-topic-name", "-j", help="Topic where the joint state is published, usually /joint_states", default="/joint_states")
    argument_parser.add_argument("--robot-name", "-r", help="name of the robot that you are calibrating, e.g. kuka_1", default="robot")
    argument_parser.add_argument("--tool-position-on-table", "-t", help="position of the tool on the table", default="unspecified_tool_position")
    argument_parser.add_argument("--joint-names", "-names", help="List of joint names to record (space-separated). If not specified, all joints from the topic will be used.", nargs='+', required=False, default=None)
    
    args = argument_parser.parse_args()
    
    joint_state_topic_name = args.joint_state_topic_name
    robot_name = args.robot_name
    tool_position = args.tool_position_on_table
    joint_names = args.joint_names
    
    try:
        # Get package path using ROS2 package discovery
        package_share_directory = get_package_share_directory('calibration_tools')
        # Navigate to workspace root and then to data folder
        data_folder = os.path.abspath(os.path.join(
            package_share_directory, 
            os.pardir, os.pardir, os.pardir, os.pardir, os.pardir,  # Go up to workspace root
            'data', robot_name, tool_position
        ))
    except Exception as e:
        # Fallback: use relative path from current directory
        print(f"Warning: Could not find package share directory, using fallback path. Error: {e}")
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_folder = os.path.abspath(os.path.join(
            current_dir, 
            os.pardir, os.pardir, os.pardir, os.pardir, os.pardir,
            'data', robot_name, tool_position
        ))
    
    print(f"Data folder: {data_folder}")
    print(f"Joint state topic: {joint_state_topic_name}")
    print(f"Robot name: {robot_name}")
    print(f"Tool position: {tool_position}")
    if joint_names is not None:
        print(f"Joint names: {joint_names}")
    else:
        print("Joint names: Will use all joints from topic")
        print("DOF: Will be determined from topic")
    
    # Create and run the node
    node = JointStatesRecorder(joint_state_topic_name, data_folder, joint_names)
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()