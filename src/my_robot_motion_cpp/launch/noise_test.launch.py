from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot",
            package_name="my_robot_moveit_config"
        ).to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="my_robot_motion_cpp",
            executable="noise_test_node",
            output="screen",
            parameters=[
                moveit_config.to_dict()
            ],
        )
    ])