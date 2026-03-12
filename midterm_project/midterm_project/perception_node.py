#!/usr/bin/env python3
"""perception_node — Stub. Will be implemented in subsequent phases."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Perception_node(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.create_subscription(
            String, '/midterm/drone_state', self.state_callback, 10)
        self.get_logger().info('perception_node initialized (stub)')

    def state_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Perception_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
