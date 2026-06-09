#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


class TestMove(Node):

    def __init__(self):
        super().__init__("test_move_node")

        self.get_logger().info("Starting MoveItPy...")

        self.moveit = MoveItPy(node_name="moveit_py")

        self.arm = self.moveit.get_planning_component("arm_group")

        self.get_logger().info("MoveItPy initialized")


    def move_to_pose(self):

        pose_goal = PoseStamped()

        pose_goal.header.frame_id = "base_link"

        pose_goal.pose.position.x = 0.30
        pose_goal.pose.position.y = 0.00
        pose_goal.pose.position.z = 0.40

        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.707
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.707

        self.arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link="end_effector_link"
        )

        self.get_logger().info("Planning trajectory...")

        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info("Plan successful")
        else:
            self.get_logger().error("Planning failed")


def main():

    rclpy.init()

    node = TestMove()

    node.move_to_pose()

    rclpy.shutdown()


if __name__ == "__main__":
    main()