from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paths to your specific files
    urdf_path = os.path.expanduser('~/gp_arm_abd/src/my_robot_description/urdf/final2.xml')
    yaml_path = os.path.expanduser('~/gp_arm_abd/src/my_robot_moveit_config/config/clean_controllers.yaml')

    return LaunchDescription([
        # 1. Start the Controller Manager
        # We only pass the robot_description here. 
        # We do NOT pass the yaml file here to avoid parameter namespace conflicts.
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),
        
        # 2. Spawn the joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--param-file', yaml_path],
        ),
        
        # 3. Spawn the arm_controller (Force-loading the YAML file directly)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager', '--param-file', yaml_path],
        ),
    ])