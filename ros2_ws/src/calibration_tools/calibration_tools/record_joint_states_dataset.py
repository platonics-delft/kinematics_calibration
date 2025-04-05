#!/usr/bin/env python3
import argparse
from pathlib import Path

import rclpy
from calibration_tools.joint_state_recorder import JointStatesRecorder


def main():
    argument_parser = argparse.ArgumentParser(description="Run the parameter optimizer")
    argument_parser.add_argument(
        "--joint-state-topic-name",
        "-j",
        help="Topic where the joint state is published",
        default="/joint_states",
    )
    argument_parser.add_argument(
        "--robot-name",
        "-r",
        help="name of the robot that you are calibrating, e.g. kuka_1",
        required=True,
    )
    argument_parser.add_argument(
        "--tool-position-on-table",
        "-t",
        help="postion of the tool on the table for the franka",
        default="front",
    )
    argument_parser.add_argument(
        "--robot-dof", "-dof", help="Degree of freedom of the robot", default=7
    )
    args = argument_parser.parse_args()
    joint_state_topic_name = args.joint_state_topic_name
    robot_name = args.robot_name
    tool_position = args.tool_position_on_table
    dof = int(args.robot_dof)

    current_package = Path(__file__).resolve().parent.parent
    data_folder = (
        current_package.parent.parent.parent / "data" / robot_name / tool_position
    )

    print("===================================================================")
    print("Data folder:", data_folder)
    print("Joint_state", joint_state_topic_name)
    print("Robot name:", robot_name)
    print("===================================================================")

    rclpy.init(args=None)
    joint_state_recorder = JointStatesRecorder(
        joint_state_topic_name=joint_state_topic_name,
        folder_name=str(data_folder),
        dof=dof,
    )
    joint_state_recorder.run()


if __name__ == "__main__":
    main()
