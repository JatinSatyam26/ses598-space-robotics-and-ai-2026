import rclpy
from rclpy.node import Node


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.get_logger().info('MappingNode initialized')


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
