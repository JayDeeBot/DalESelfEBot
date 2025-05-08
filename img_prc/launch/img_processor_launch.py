from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
     
    return LaunchDescription([

        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Index of the webcam to use'
        ),

        # First node (talker)
        Node(
            package='img_prc',
            executable='webcam_pub',
            name='webcam_publisher',
            output='screen',
            arguments=[LaunchConfiguration('camera_index')]
        ),

        # Second node (listener) with argument passed
        Node(
            package='img_prc',
            executable='edge_pub',
            name='edge_publisher',
            output='screen',
        ),

        Node(
            package='img_prc',
            executable='bg_rmv_server',
            name='bg_rmv_server',
            output='screen',
        ),

    ])

