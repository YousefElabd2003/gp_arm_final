#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class LeverTestBridge(Node):

    def __init__(self):

        super().__init__("lever_test_bridge")

        self.subscription = self.create_subscription(
            PoseStamped,
            "/lever_pose_arm",
            self.pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            "/lever_target",
            10
        )

        self.target_locked = False

        self.get_logger().info(
            "Lever Test Bridge Ready"
        )

    def pose_callback(self, msg):

        if self.target_locked:
            return

        self.target_locked = True

        target = PoseStamped()

        target.header = msg.header

        target.pose.position.x = msg.pose.position.x
        target.pose.position.y = msg.pose.position.y
        target.pose.position.z = msg.pose.position.z

        target.pose.orientation.x = msg.pose.orientation.x
        target.pose.orientation.y = msg.pose.orientation.y
        target.pose.orientation.z = msg.pose.orientation.z
        target.pose.orientation.w = msg.pose.orientation.w

        self.publisher.publish(target)

        self.get_logger().info(
            f"Target forwarded: "
            f"x={target.pose.position.x:.3f} "
            f"y={target.pose.position.y:.3f} "
            f"z={target.pose.position.z:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LeverTestBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()