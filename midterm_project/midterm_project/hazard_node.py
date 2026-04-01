import rclpy
from rclpy.node import Node


class HazardNode(Node):
    def __init__(self):
        super().__init__('hazard_node')
        self.get_logger().info('HazardNode initialized')


def main(args=None):
    rclpy.init(args=args)
    node = HazardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
