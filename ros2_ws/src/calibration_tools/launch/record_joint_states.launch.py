from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_state_topic_name',
            default_value='/joint_states',
            description='Topic where the joint state is published'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='panda_1',
            description='Name of the robot that you are calibrating, e.g. kuka_1'
        ),
        DeclareLaunchArgument(
            'tool_position_on_table',
            default_value='center',
            description='Position of the tool on the table for the franka'
        ),
        DeclareLaunchArgument(
            'robot_dof',
            default_value='7',
            description='Degree of freedom of the robot'
        ),
        
        Node(
            package='calibration_tools',
            executable='record_joint_states_dataset',
            name='joint_states_recorder',
            parameters=[{
                'joint_state_topic_name': LaunchConfiguration('joint_state_topic_name'),
                'robot_name': LaunchConfiguration('robot_name'),
                'tool_position_on_table': LaunchConfiguration('tool_position_on_table'),
                'robot_dof': LaunchConfiguration('robot_dof'),
            }],
            output='screen'
        )
    ])