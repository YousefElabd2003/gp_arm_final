import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    robot_name = "my_robot"
    moveit_config_pkg = "my_robot_moveit_config"
    
    ros2_controllers_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'ros2_controllers.yaml')
    moveit_controllers_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'moveit_controllers.yaml')
    sensors_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'sensors_3d.yaml')
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_pkg)
        .trajectory_execution(file_path=ros2_controllers_path)
        .sensors_3d(file_path=sensors_path)
        .to_moveit_configs()
    )

    return LaunchDescription([
        # 1. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        # 2. RViz (Respawning for WSL2 stability)
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'moveit.rviz')],
            parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
            respawn=True, respawn_delay=2.0,
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe'}
        ),
        # 3. Spawner
        Node(package='ros_gz_sim', executable='create', arguments=['-name', robot_name, '-topic', 'robot_description']),
        # 4. RSP
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[moveit_config.robot_description, {'use_sim_time': True}]),
        # 5. Bridge
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        # 6. MoveGroup (Linked to moveit_controllers.yaml)
        Node(
            package='moveit_ros_move_group', executable='move_group',
            parameters=[
                moveit_config.to_dict(), 
                {'use_sim_time': True},
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                moveit_controllers_path
            ]
        ),
        # 7. Static TF
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']),
        # 8. JSB Spawner
        TimerAction(period=8.0, actions=[Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '-c', '/controller_manager'])]),
        # 9. Arm Controller
        TimerAction(period=10.0, actions=[Node(package='controller_manager', executable='spawner', arguments=['arm_controller', '-c', '/controller_manager'])])
    ])