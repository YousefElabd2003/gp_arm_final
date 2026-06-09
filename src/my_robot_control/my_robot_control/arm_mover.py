import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_mover_node')
        # We are switching to 'move_action' based on your 'ros2 action list -t'
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Action Client pointing to /move_action.')

    def send_goal(self):
        self.get_logger().info('Waiting for server...')
        # Increased timeout for WSL2 discovery
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Could not find /move_action. Trying /move_group...')
            # Fallback attempt
            self._action_client = ActionClient(self, MoveGroup, 'move_group')
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Both interfaces failed. Check network exports.')
                return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.pipeline_id = "ompl"

        # Goal Position
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.1
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.1
        target_pose.pose.orientation.w = 1.0

        # Constraints
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
        goal_msg.planning_options.plan_only = False

        self.get_logger().info('Sending goal now!')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionClient()
    node.send_goal()
    rclpy.spin_once(node)
    # Give it a second to actually send the packet before shutting down
    import time
    time.sleep(1) 
    rclpy.shutdown()

if __name__ == '__main__':
    main()