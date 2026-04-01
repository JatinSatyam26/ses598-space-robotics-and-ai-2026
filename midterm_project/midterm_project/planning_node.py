import rclpy
from rclpy.node import Node


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        self.get_logger().info('PlanningNode initialized')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
