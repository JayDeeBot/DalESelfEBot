from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    # Path to the YAML configuration file
    config_path = '/home/jarred/git/DalESelfEBot/GUI/params.yaml'  # Update this path accordingly

    # Load the YAML file
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            ur_type = config['robot']['ur_type']
            ip_address = config['robot']['ip_address']
    except Exception as e:
        raise RuntimeError(f"Failed to load 'ur_type' from YAML: {e}")

    return LaunchDescription([

        # # UR Driver
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('ur_robot_driver'),
        #             'launch',
        #             'ur_control.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'ur_type': ur_type,
        #         'robot_ip': ip_address,
        #         'launch_rviz': 'true',
        #         'kinematics_config': '/home/jarred/git/DalESelfEBot/GUI/calibration.yaml'
        #     }.items()
        # ),

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

        # Image Processing Node
        Node(
            package='img_prc',
            executable='image_processor',
            name='image_processor',
            output='screen'
        ),

        # Additional nodes or launch files can be added here
    ])
