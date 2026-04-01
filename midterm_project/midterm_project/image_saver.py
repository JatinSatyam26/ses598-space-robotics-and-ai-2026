import rclpy
from rclpy.node import Node


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.get_logger().info('ImageSaver initialized')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
