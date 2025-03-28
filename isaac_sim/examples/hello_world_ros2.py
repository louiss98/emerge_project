# hello_world_ros2.py
#
# This script demonstrates a simple ROS 2 publisher that publishes a "Hello World" message to a topic.
#
# Follow the steps below to execute this script in your WSL (Windows Subsystem for Linux) or VMware Linux environment:
#
# ====================
# WSL (Windows Subsystem for Linux) Execution:
# ====================
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
# =========================
# VMware (Virtual Machine) Execution:
# =========================
# 1. Start your VMware virtual machine (VM) with a Linux distribution (e.g., Ubuntu).
# 2. Open a terminal inside the VM.
#
# 3. If the script is on your Windows host, ensure that you have shared the folder between the host and VM.
#    Then, mount the shared folder in your VM (e.g., /mnt/hgfs).
#    Example command to mount shared folder:
#    sudo mount -t vmhgfs .host:/Shared /mnt/hgfs
#    cd /mnt/hgfs/ccsu_s2r_project  # Navigate to the directory where the script is stored.
#
# 4. Ensure that you have ROS 2 installed on your VM. If you haven't installed ROS 2 yet, follow the
#    instructions for installing ROS 2 Humble in Ubuntu: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#
# 5. Source the ROS 2 setup files in your terminal:
#    source /opt/ros/humble/setup.bash
#
# 6. Run the script using Python 3 in your VM terminal:
#    python3 hello_world_ros2.py
#
# ====================
# Expected Output:
# ====================
# If everything is set up correctly, the script will start publishing a "Hello World" message to the topic
# 'hello_world_topic' every second. You should see the message being printed in the terminal:
#    Publishing: "Hello World"

# E.M.E.R.G.E – Emulation of Mobility and Expressiveness in Robotic Quadrupeds for Global Environments 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import isaacsim.ros2.bridge
print(isaacsim.ros2.bridge.__file__)

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0  # Track number of messages published
        self.max_count = 10  # Stop after 10 messages

    def timer_callback(self):
        """Publishes messages and stops after 10 messages."""
        if self.count < self.max_count:
            msg = String()
            msg.data = f'Hello World {self.count + 1}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.count += 1
        else:
            self.get_logger().info("Reached max count, stopping...")
            stop_ros_node()  # Gracefully stop execution


# Global stop event and ROS thread
stop_event = threading.Event()
ros_thread = None


def run_ros2_node():
    """Runs the ROS2 node in a background thread."""
    global stop_event

    # Initialize ROS only if it's not already initialized
    if not rclpy.utilities.ok():
        print("Initializing ROS2...")
        rclpy.init()
    else:
        print("ROS2 is already initialized, skipping duplicate initialization.")

    node = HelloWorldPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down ROS2...")
        node.destroy_node()
        rclpy.shutdown()
        print("ROS2 shut down successfully.")


def start_ros_node():
    """Starts the ROS2 node in a new thread."""
    global ros_thread
    if ros_thread is None or not ros_thread.is_alive():
        stop_event.clear()
        ros_thread = threading.Thread(target=run_ros2_node, daemon=True)
        ros_thread.start()
        print("ROS2 node started.")


def stop_ros_node():
    """Stops the ROS2 node."""
    global stop_event, ros_thread

    if not stop_event.is_set():
        print("Stopping ROS2 node...")
        stop_event.set()

    if ros_thread and ros_thread.is_alive():
        ros_thread.join()
        print("ROS2 node stopped.")

    # Ensure rclpy is fully shutdown
    if rclpy.ok():
        rclpy.shutdown()


# Start the ROS2 node
start_ros_node()
