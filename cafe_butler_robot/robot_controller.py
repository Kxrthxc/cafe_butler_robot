import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from enum import Enum
from collections import deque


class RobotState(Enum):
    HOME = 0
    GO_TO_KITCHEN = 1
    WAIT_KITCHEN_CONFIRM = 2
    GO_TO_TABLE = 3
    WAIT_TABLE_CONFIRM = 4
    RETURN_TO_KITCHEN = 5
    RETURN_HOME = 6


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.state = RobotState.HOME
        self.order_queue = deque()
        self.current_table = None
        self.timeout_duration = 8.0
        self.start_time = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(String, '/dispatch_order', self.order_callback, 10)
        self.create_subscription(String, '/cancel_order', self.cancel_callback, 10)
        self.create_subscription(Bool, '/kitchen_confirm', self.kitchen_confirm, 10)
        self.create_subscription(Bool, '/table_confirm', self.table_confirm, 10)

        self.timer = self.create_timer(1.0, self.update_state)

        self.locations = {
            "home"   : (0.0, -1.5),
            "kitchen": (1.5, 0.0)
            ,
            "table1" : (2.0, 2.0),
            "table2" : (-2.0, 2.0),
            "table3" : (0.0, -2.0),
        }

    # -------------------------
    # Topic Callbacks
    # -------------------------

    def order_callback(self, msg):
        self.order_queue.append(msg.data)
        self.get_logger().info(f"Order received: {msg.data}")

    def cancel_callback(self, msg):
        if msg.data in self.order_queue:
            self.order_queue.remove(msg.data)
            self.get_logger().info(f"Order cancelled: {msg.data}")

        if self.current_table == msg.data:
            self.get_logger().info("Cancelling current delivery")
            self.current_table = None
            self.state = RobotState.RETURN_TO_KITCHEN
            self.go_to("kitchen")

    def kitchen_confirm(self, msg):
        if self.state == RobotState.WAIT_KITCHEN_CONFIRM and msg.data:
            self.get_logger().info("Kitchen confirmed pickup")
            self.state = RobotState.GO_TO_TABLE

    def table_confirm(self, msg):
        if self.state == RobotState.WAIT_TABLE_CONFIRM and msg.data:
            self.get_logger().info("Table confirmed delivery")
            self.current_table = None

            if self.order_queue:
                self.state = RobotState.GO_TO_TABLE
            else:
                self.state = RobotState.RETURN_TO_KITCHEN
                self.go_to("kitchen")

    # -------------------------
    # Navigation
    # -------------------------

    def go_to(self, location):

        if location not in self.locations:
            self.get_logger().error(f"Unknown location: {location}")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.locations[location][0]
        pose.pose.position.y = self.locations[location][1]
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected")
            return

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        if self.state == RobotState.GO_TO_KITCHEN:
            self.state = RobotState.WAIT_KITCHEN_CONFIRM
            self.start_time = self.get_clock().now()

        elif self.state == RobotState.GO_TO_TABLE:
            self.state = RobotState.WAIT_TABLE_CONFIRM
            self.start_time = self.get_clock().now()

        elif self.state == RobotState.RETURN_TO_KITCHEN:
            self.state = RobotState.RETURN_HOME
            self.go_to("home")

        elif self.state == RobotState.RETURN_HOME:
            self.state = RobotState.HOME
            self.get_logger().info("Returned Home")

    # -------------------------
    # State Machine
    # -------------------------

    def update_state(self):

        if self.state == RobotState.HOME and self.order_queue:
            self.get_logger().info("Going to kitchen for pickup")
            self.state = RobotState.GO_TO_KITCHEN
            self.go_to("kitchen")

        elif self.state == RobotState.GO_TO_TABLE:

            if self.current_table is None:
                if self.order_queue:
                    self.current_table = self.order_queue.popleft()
                else:
                    return

            self.go_to(self.current_table)

        elif self.state == RobotState.WAIT_KITCHEN_CONFIRM:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.timeout_duration:
                self.get_logger().info("Kitchen confirmation timeout")
                self.state = RobotState.RETURN_HOME
                self.go_to("home")

        elif self.state == RobotState.WAIT_TABLE_CONFIRM:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.timeout_duration:
                self.get_logger().info("Table confirmation timeout")
                self.current_table = None

                if self.order_queue:
                    self.state = RobotState.GO_TO_TABLE
                else:
                    self.state = RobotState.RETURN_TO_KITCHEN
                    self.go_to("kitchen")


# -------------------------
# Main
# -------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

