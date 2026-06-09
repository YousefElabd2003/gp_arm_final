import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point
import serial
import math

class ArmPerceptionBridge(Node):
    def __init__(self):
        super().__init__('arm_perception_bridge')
        
        # 1. Serial setup for your hardware
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Success: Hardware Serial Connected.')
        except Exception as e:
            self.get_logger().error(f'Hardware connection failed: {e}')

        # 2. Subscriber for your Stereo Camera Script
        # Make sure your stereo script publishes to '/detected_object_3d'
        self.subscription = self.create_subscription(
            Point,
            '/detected_object_3d',
            self.coordinate_callback,
            10)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Bridge Ready. Awaiting Stereo Camera 3D coordinates...')

    def coordinate_callback(self, msg):
        self.get_logger().info(f'New Target Detected: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
        self.send_goal(msg.x, msg.y, msg.z)

    def send_goal(self, x, y, z):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup server offline!')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link" # Ensure coordinates are relative to arm base
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0

        # Standard MoveIt IK Constraint block
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "j6_arm_link" 
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.02] # 2cm tolerance for perception noise
        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(target_pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(constraints)

        # Trigger Planning and Execution
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('MoveIt rejected the 3D target.')
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.planned_trajectory.joint_trajectory.points:
            # Extract final solved angles
            final_point = result.planned_trajectory.joint_trajectory.points[-1]
            angles_deg = [round(math.degrees(p), 2) for p in final_point.positions]
            
            # Bridge to ESP32-S3
            payload = " ".join(map(str, angles_deg[:6])) + "\n"
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(payload.encode('utf-8'))
                self.get_logger().info(f'IK Executed. Hardware Payload: {payload.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmPerceptionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()