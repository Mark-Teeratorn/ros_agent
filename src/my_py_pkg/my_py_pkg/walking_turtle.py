import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from turtlesim.srv import TeleportAbsolute  # Import the service type

class WalkingTurtleNode(Node):
    def __init__(self):
        super().__init__('walking_turtle_node')
        self.position = 0
        self.subscription = self.create_subscription(
            Int32,  # Message type
            'count_topic',  # Topic name (updated to match count.py)
            self.listener_callback,  # Callback function
            10  # QoS depth
        )
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')  # Create service client
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute service...')
        self.get_logger().info('Walking Turtle Node has been started.')

    def listener_callback(self, msg):
        self.position = msg.data  # Update position based on received message
        self.get_logger().info(f'I heard: "{self.position}"')
        self.teleport_turtle(self.position)

    def teleport_turtle(self, position):
        # Create a request for the teleport_absolute service
        request = TeleportAbsolute.Request()
        request.x = float(position)  # Set the x-coordinate based on the received position
        request.y = 0.0  # Fixed y-coordinate
        request.theta = 0.0  # Fixed orientation

        # Call the service
        future = self.client.call_async(request)
        future.add_done_callback(self.teleport_callback)

    def teleport_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Turtle teleported successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to teleport turtle: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WalkingTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


