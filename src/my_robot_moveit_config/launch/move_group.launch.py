from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config").to_moveit_configs()
    
    # Manually define the path to your controller configuration
    controllers_yaml = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config",
        "moveit_controllers.yaml"
    )

    # Generate the standard move_group launch
    ld = generate_move_group_launch(moveit_config)
    
    # We need to ensure the parameters are passed correctly
    # You may need to inject the params manually if the helper doesn't pick them up
    return ld