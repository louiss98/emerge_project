import threading
import asyncio
import omni.usd
import omni.kit.commands
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import omni.kit.app
import time
from pxr import Sdf

# Use the new SingleArticulation API instead of the deprecated Articulation
from isaacsim.core.prims import SingleArticulation as Articulation

# Get the environment context and stage
usd_context = omni.usd.get_context()
stage = usd_context.get_stage()
timeline = omni.timeline.get_timeline_interface()
stop_event = threading.Event()

# Define the prim path where the robot will be spawned and the asset path
robot_prim_path = "/World/UnitreeGo2"
robot_asset_path = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/"
                    "Isaac/Robots/Unitree/Go2/go2.usd")

# Ensure the physics scene exists
physics_scene_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_scene_path):
    print("Creating PhysicsScene prim at", physics_scene_path)
    omni.kit.commands.execute(
        "CreatePrim",
        prim_type="PhysicsScene",
        prim_path=physics_scene_path
    )
else:
    print("PhysicsScene already exists at", physics_scene_path)

time.sleep(1)

# Spawn the robot if not already present
if not stage.GetPrimAtPath(robot_prim_path):
    print("Spawning Unitree GO2 robot at prim path:", robot_prim_path)
    omni.kit.commands.execute(
        "CreateReference",
        usd_context=usd_context,
        path_to=robot_prim_path,
        asset_path=robot_asset_path
    )
    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=Sdf.Path(f"{robot_prim_path}.physics:enableArticulation"),
        value=True,
        prev=None
    )
    omni.kit.commands.execute(
        "AddArticulationRoot",
        path=robot_prim_path
    )
else:
    print("Robot is already spawned at prim path:", robot_prim_path)

# Start the simulation timeline
timeline.play()

# Create a SingleArticulation instance for the robot using the new API.
articulation = Articulation(robot_prim_path)
print("Articulation instance created:", articulation)

# Define the joints you expect (adjust as necessary)
joints = [
    "FL_hip_joint",
    "FR_hip_joint",
    "RL_hip_joint",
    "RR_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_thigh_joint",
    "RR_calf_joint"
]
num_joints = len(joints)
print(f"Robot has {num_joints} joints.")

# Define a ROS2 node that publishes joint states using the new Articulation API.
class JointStatePublisher(Node):
    """ROS2 node that publishes joint states."""
    def __init__(self, articulation):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'unitree_go2/joint_states', 10)
        self.articulation = articulation
        self.start_time = time.time()
        self.duration = 10  # seconds
        self.timer_callback()  # Start publishing loop

    def timer_callback(self):
        if time.time() - self.start_time > self.duration:
            self.get_logger().info("Finished publishing joint states.")
            stop_event.set()  # Signal the main loop to stop
            return

        # For testing purposes, explicitly set joint values:
        explicit_positions = [0.1 * i for i in range(num_joints)]
        # Using the new Articulation API to set joint positions
        self.articulation.set_joint_positions(explicit_positions)

        # Retrieve the joint positions from the robot articulation
        positions = self.articulation.get_joint_positions()

        if positions is None:
            self.get_logger().warn("Articulation positions are None. Check initialization.")
            return  # Exit the callback

        # Ensure each value is a plain Python float
        positions = [float(p) for p in positions]

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = joints
        joint_state_msg.position = positions

        self.publisher_.publish(joint_state_msg)
        self.get_logger().info(f"Published joint states: {positions}")
        print("Joint state data:", positions)

        # Schedule the next call to timer_callback (non-blocking)
        threading.Timer(0.5, self.timer_callback).start()

def ros2_spin(node):
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        print("Starting ROS2 spinning...")
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down ROS2...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("ROS2 shut down successfully.")

def run_ros2_node():
    if not rclpy.utilities.ok():
        print("Initializing ROS2...")
        rclpy.init()
    node = JointStatePublisher(articulation)
    ros2_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    ros2_thread.start()
    print("ROS2 node started in background thread.")

# Start the ROS2 node in a background thread
ros_thread = threading.Thread(target=run_ros2_node, daemon=True)
ros_thread.start()
print("ROS2 node started.")

# Keep the simulation running
try:
    while not stop_event.is_set():
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    print("Shutting down ROS2...")
    rclpy.shutdown()
    print("ROS2 shut down successfully.")
    timeline.stop()
    print("Timeline stopped.")