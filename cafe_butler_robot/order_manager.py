import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderManager(Node):

    def __init__(self):
        super().__init__('order_manager')

        self.publisher = self.create_publisher(String, '/dispatch_order', 10)

        self.subscription = self.create_subscription(
            String,
            '/new_order',
            self.order_callback,
            10)

    def order_callback(self, msg):
        self.get_logger().info(f"Dispatching {msg.data}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

