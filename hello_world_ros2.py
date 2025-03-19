import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Define a HelloWorldPublisher class that inherits from Node
class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')  # Initialize the node with the name 'hello_world_publisher'
        self.publisher_ = self.create_publisher(String, 'hello_world_topic', 10)  # Create a publisher for the 'hello_world_topic' with a queue size of 10
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Create a timer that calls the timer_callback function every second

    def timer_callback(self):
        msg = String()  # Create a new String message
        msg.data = 'Hello World'  # Set the data of the message to 'Hello World'
        self.publisher_.publish(msg)  # Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data)  # Log the message data

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = HelloWorldPublisher()  # Create an instance of the HelloWorldPublisher node
    try:
        rclpy.spin(node)  # Keep the node running, allowing it to process callbacks
    except KeyboardInterrupt:
        pass  # Handle the KeyboardInterrupt exception to allow for graceful shutdown
    node.destroy_node()  # Destroy the node explicitly
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library

if __name__ == '__main__':
    main()  # Run the main function if the script is executed directly
