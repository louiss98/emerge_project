# hello_world_ros2.py
#
# This script demonstrates a simple ROS 2 publisher that publishes a "Hello World" message to a topic.
#
# Follow the steps below to execute this script in your WSL (Windows Subsystem for Linux) environment:
#
# 1. Open your WSL terminal (Linux environment).
# 2. Navigate to the directory where the Python script is located. 
#    For example, if the script is located on your D: drive in Windows, use the following command:
#    cd /mnt/d/ccsu_s2r_project
#
# 3. Ensure that you have ROS 2 installed in your WSL environment. If you haven't installed ROS 2 yet,
#    follow the instructions for installing ROS 2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#
# 4. Source the ROS 2 setup files:
#    source /opt/ros/humble/setup.bash
#
# 5. Run the script using Python 3 in your WSL terminal:
#    python3 hello_world_ros2.py
#
# If everything is set up correctly, the script will start publishing a "Hello World" message every second.

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
