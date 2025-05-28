from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
import yaml
from launch.actions import TimerAction

def generate_launch_description():
    # Path to the YAML configuration file
    config_path = '/home/jarred/git/DalESelfEBot/GUI/params.yaml'  # Update this path accordingly

    # Load the YAML file
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            ur_type = config['robot']['ur_type']
            camera_index = config['robot']['camera_index']
    except Exception as e:
        raise RuntimeError(f"Failed to load 'ur_type' from YAML: {e}")

    return LaunchDescription([
        # Motion Planning (MoveIt config)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py'
                ])
            ]),
            launch_arguments={'ur_type': ur_type, 'launch_rviz': 'false'}.items()
        ),

        # Pause for 5 seconds before starting the rest
        TimerAction(
            period=5.0,
            actions=[]
        ),

        # Motion Execution (Control stack)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur3_control'),
                    'launch',
                    'moveit_stack.launch.py'
                ])
            ]),
            launch_arguments={'ur_type': ur_type, 'launch_rviz': 'false'}.items()
        ),

        # Image Processor Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('img_prc'),
                    'launch',
                    'img_processor_launch.py'
                ])
            ]),
            launch_arguments={'camera_index': camera_index}.items()
        ),

        # Toolpath Planner Node
        Node(
            package='tool_path_planning',
            executable='tool_path_planner_node',
            name='tool_path_planner_node',
            output='screen'
        ),
    ])