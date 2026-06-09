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

    serial_bridge_node = Node(
        package="my_robot_motion_cpp",
        executable="serial_bridge_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"simulation_mode": True},           # Set to False for real hardware
            {"serial_port": "/dev/ttyUSB0"},
            {"baud_rate": 115200},
            {"approach_z": 0.950},               # Approach height (above lever)
            {"approach_offset_y": 0.20},         # Y offset for approach (20cm)
            {"pull_distance_x": 0.05},           # X pull distance (5cm)
        ],
    )

    return LaunchDescription([serial_bridge_node])