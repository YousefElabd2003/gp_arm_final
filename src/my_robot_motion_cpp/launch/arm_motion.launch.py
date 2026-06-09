from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot",
            package_name="my_robot_moveit_config"
        )
        .to_moveit_configs()
    )

    arm_motion_node = Node(
        package="my_robot_motion_cpp",
        executable="arm_motion_node",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    return LaunchDescription([
        arm_motion_node
    ])