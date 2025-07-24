import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Import Twist message

class CommandNode(Node):
    def __init__(self):
        super().__init__("command_turtle")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Starting Command Publisher")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0   # Move forward
        msg.linear.y = 1.0   # This will be ignored by turtlesim
        msg.angular.z = 1.0  # Rotate
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published velocity command: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

