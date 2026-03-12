#!/usr/bin/env python3
"""planning_node — Stub. Will be implemented in subsequent phases."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Planning_node(Node):
    def __init__(self):
        super().__init__('planning_node')
        self.create_subscription(
            String, '/midterm/drone_state', self.state_callback, 10)
        self.get_logger().info('planning_node initialized (stub)')

    def state_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Planning_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
