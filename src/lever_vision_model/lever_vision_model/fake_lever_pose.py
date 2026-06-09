#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class FakeLeverPublisher(Node):

    def __init__(self):

        super().__init__("fake_lever_publisher")

        self.publisher_ = self.create_publisher(
            PoseStamped,
            "/lever_pose_arm",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_pose
        )

        self.get_logger().info(
            "Publishing fake lever pose..."
        )

    def publish_pose(self):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "Arm_base_link"

        # =====================================
        # Same pose that worked in arm_motion_node
        # =====================================

        msg.pose.position.x = 0.900
        msg.pose.position.y = 0.024
        msg.pose.position.z = 0.902

        # =====================================
        # LEFT lever flag
        # Coordinator checks orientation.x
        # =====================================

        msg.pose.orientation.x = -1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Publishing: "
            f"x={msg.pose.position.x:.3f} "
            f"y={msg.pose.position.y:.3f} "
            f"z={msg.pose.position.z:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = FakeLeverPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()