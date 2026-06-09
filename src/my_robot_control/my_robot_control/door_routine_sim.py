import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume, JointConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import math
import copy
import time

class DoorLeverSimulator(Node):
    def __init__(self):
        super().__init__('door_routine_simulator')
        
        self._move_action = ActionClient(self, MoveGroup, 'move_action')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._execute_action = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        self.joint_names = [
            'j1_arm_joint', 'j2_arm_joint', 'j3_arm_joint', 
            'j4_arm_joint', 'j5_arm_joint', 'j6_arm_joint'
        ]
        self.home_angles = [0.0, -0.50, 1.20, -0.70, 0.0, 0.0]
        self.move_group_name = "arm" 

    def execute_test(self, lever_x, lever_y, lever_z):
        self.get_logger().info('================================')
        self.get_logger().info('   GAZEBO DOOR LEVER TEST       ')
        self.get_logger().info('================================')

        self.get_logger().info('Phase 0: Moving to floating Home position...')
        if not self.move_to_home():
            self.get_logger().error('Failed to initialize at Home.')
            return
        time.sleep(1.0)

        # Pre-calculate hover pose so the recovery block can use it if needed
        hover_pose = Pose()
        hover_pose.position.x = lever_x
        hover_pose.position.y = lever_y
        hover_pose.position.z = lever_z + 0.10 # Hover 10cm exactly above
        hover_pose.orientation.x = 0.0
        hover_pose.orientation.y = 0.7071
        hover_pose.orientation.z = 0.0
        hover_pose.orientation.w = 0.7071

        try:
            # PHASE 1A: HOVER
            self.get_logger().info('Phase 1A: Aligning directly above the lever...')
            if not self.move_to_pose(hover_pose, enforce_orientation=False):
                raise RuntimeError("Phase 1A (Hover) Failed. Target is likely out of reach.")
            time.sleep(1.0) 

            # PHASE 1B: Z-DROP
            self.get_logger().info('Phase 1B: Executing strict Z-transform onto the lever...')
            grip_pose = copy.deepcopy(hover_pose)
            grip_pose.position.z = lever_z 
            
            if not self.move_to_pose(grip_pose, enforce_orientation=True):
                raise RuntimeError("Phase 1B (Z-Drop) Failed. Kinematic lock during descent.")
            time.sleep(1.0) 

            # PHASE 2: CARTESIAN ARC
            self.get_logger().info('Phase 2: Calculating downward Cartesian Arc...')
            waypoints = []
            wpose = copy.deepcopy(grip_pose)
            R = 0.10 
            max_angle = 45 
            steps = 10
            
            for i in range(1, steps + 1):
                theta = math.radians((max_angle / steps) * i)
                wpose.position.z = grip_pose.position.z - (R * math.sin(theta))
                wpose.position.x = grip_pose.position.x + (R * (1 - math.cos(theta)))
                waypoints.append(copy.deepcopy(wpose))

            req = GetCartesianPath.Request()
            req.header.frame_id = 'base_link'
            req.group_name = self.move_group_name
            req.link_name = 'j6_arm_link'
            req.waypoints = waypoints
            req.max_step = 0.01 

            self._cartesian_srv.wait_for_service()
            future = self._cartesian_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.fraction < 0.9:
                raise RuntimeError(f"Phase 2 (Arc) Failed. Only solved {response.fraction * 100}%.")
                
            self.get_logger().info('Arc solved! Executing pull...')
            self.execute_trajectory(response.solution)
            time.sleep(1.0)

            # PHASE 3: PUSH DOOR
            self.get_logger().info('Phase 3: Pushing the door open...')
            push_pose = copy.deepcopy(wpose)
            push_pose.position.x += 0.15 
            if not self.move_to_pose(push_pose, enforce_orientation=False):
                 raise RuntimeError("Phase 3 (Push) Failed. Arm overextended.")
            time.sleep(1.0)

        except RuntimeError as e:
            # === THE RECOVERY BLOCK ===
            self.get_logger().error(f"ABORT TRIGGERED: {e}")
            self.get_logger().warn("Executing Safe Retract from current position...")
            
            # Step 1: Retract back to the elevated hover position to clear the physical door lever
            self.move_to_pose(hover_pose, enforce_orientation=False)
            
            # Step 2: Back away from the door plane by 15cm
            retreat_pose = copy.deepcopy(hover_pose)
            retreat_pose.position.x -= 0.15
            self.move_to_pose(retreat_pose, enforce_orientation=False)
            
        finally:
            # === THE HOME BLOCK ===
            # This will execute regardless of whether the routine succeeded or entered the Recovery Block
            self.get_logger().info('Phase 4: Folding back into Home position...')
            self.move_to_home()
            self.get_logger().info('Routine Complete!')


    # ... KEEP YOUR EXISTING move_to_home, move_to_pose, and execute_trajectory FUNCTIONS HERE ...
    def move_to_home(self):
        self._move_action.wait_for_server()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        
        constraints = Constraints()
        for i, j_name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = self.home_angles[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._move_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def move_to_pose(self, target_pose, enforce_orientation=False):
        self._move_action.wait_for_server()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        
        constraints = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = "base_link" 
        pc.link_name = "j6_arm_link"    
        s = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        bv = BoundingVolume(primitives=[s], primitive_poses=[target_pose])
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        if enforce_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = "base_link"
            oc.link_name = "j6_arm_link"
            oc.orientation = target_pose.orientation
            oc.absolute_x_axis_tolerance = 0.05 
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = 0.05
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._move_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def execute_trajectory(self, trajectory):
        self._execute_action.wait_for_server()
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = self._execute_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


def main(args=None):
    rclpy.init(args=args)
    node = DoorLeverSimulator()
    
    # Try testing a coordinate that you know will force Phase 2 to fail (like x=0.50)
    # to watch the new Safe Retract sequence pull the arm out gracefully!
    sim_x = 0.70 
    sim_y = 0.00
    sim_z = 0.70 
    
    node.execute_test(sim_x, sim_y, sim_z)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()