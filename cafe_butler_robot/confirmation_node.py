import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ConfirmationNode(Node):

    def __init__(self):
        super().__init__('confirmation_node')

        self.kitchen_pub = self.create_publisher(Bool, '/kitchen_confirm', 10)
        self.table_pub = self.create_publisher(Bool, '/table_confirm', 10)

        self.timer = self.create_timer(10.0, self.send_confirmation)

    def send_confirmation(self):
        msg = Bool()
        msg.data = True
        self.kitchen_pub.publish(msg)
        self.table_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConfirmationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

