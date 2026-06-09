import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import serial
import math
import time

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_mover_node')
        
        # 1. Initialize Serial for ESP32-S3
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Connected to ESP32 Hardware.')
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Action Client pointing to /move_action.')

    def send_goal(self):
        self.get_logger().info('Waiting for server...')
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Could not find /move_action.')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.pipeline_id = "ompl"

        # Goal Position (X, Y, Z)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.35 # Adjusted to a reachable point
        target_pose.pose.position.y = 0.05
        target_pose.pose.position.z = 0.30
        target_pose.pose.orientation.w = 1.0

        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "j6_arm_link" 
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] 
        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(target_pose.pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(constraints)
        
        # Ensure we get the trajectory back to extract angles
        goal_msg.planning_options.plan_only = False

        self.get_logger().info('Sending goal now!')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        # 2. Add callback to process the result
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # 3. Extract the calculated IK angles from the trajectory
        if result.planned_trajectory.joint_trajectory.points:
            # We take the final point of the solved path
            final_point = result.planned_trajectory.joint_trajectory.points[-1]
            positions = final_point.positions
            
            # Convert Radians to Degrees for ESP32 PID loop
            angles_deg = [round(math.degrees(p), 2) for p in positions]
            
            # Format: "J1 J2 J3 J4 J5 J6\n"
            payload = " ".join(map(str, angles_deg[:6])) + "\n"
            
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(payload.encode('utf-8'))
                self.get_logger().info(f'Sent to ESP32: {payload.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionClient()
    node.send_goal()
    # 4. Use spin() instead of spin_once to allow callbacks to finish
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()