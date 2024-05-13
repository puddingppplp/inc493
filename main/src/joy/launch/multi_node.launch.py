from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Task 1: Including vesc_driver package
    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vesc_driver'),
                                                  'launch', 'vesc_driver_node.launch.py'))
    )

    ld.add_action(vesc_driver_launch)

    # Task 2: Including joy package
    joy_node = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )
    ld.add_action(joy_node)

    # Add more IncludeLaunchDescription actions or ExecuteProcess commands for other packages

    return ld

