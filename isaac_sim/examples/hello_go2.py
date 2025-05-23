import threading
import asyncio
import omni.usd
import omni.kit.commands
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
import time
from pxr import Sdf, Gf, UsdGeom, UsdPhysics
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
import numpy as np
from isaacsim.core.api.world import World
from omni.isaac.core.utils.stage import get_current_stage
import math
from isaacsim.core.api import SimulationContext
import omni.kit.app
from omni.kit.async_engine import run_coroutine
import queue
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from concurrent.futures import ThreadPoolExecutor
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import get_current_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
from omni.physx.scripts import utils, physicsUtils

# Get the environment context and stage
usd_context = omni.usd.get_context()
stage = usd_context.get_stage()
timeline = omni.timeline.get_timeline_interface()


stop_event = threading.Event()
physics_ready_event = threading.Event()
articulation_ready_event = threading.Event()

# Define paths
robot_prim_path = "/World/UnitreeGo2"
robot_asset_path = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/"
    "Isaac/Robots/Unitree/Go2/go2.usd")
physics_scene_path = "/World/PhysicsScene" 
robot_prim_path = "/World/UnitreeGo2"
robot_imu_prim_path = robot_prim_path + "/imu"

initialization_done = False

# Define the joints. These match the Unitree GO2.
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

# Define wave parameters
frequency = 0.5  # oscillations per second
amplitude = 0.5  # maximum joint deviation
# Create or get the world instance. The World object manages physics callbacks.

# Initialize the simulation world
world = World()

# Get the current stage
stage = get_current_stage()

def setup_env():
    if not stage.GetPrimAtPath(physics_scene_path):
        print("Creating PhysicsScene prim at", physics_scene_path)
        omni.kit.commands.execute(
            "CreatePrim",
            prim_type="PhysicsScene",
            prim_path=physics_scene_path
        )
    else:
        print("Physics Scene already exists at", physics_scene_path)

    # Spawn the Unitree GO2 if not already present
    if not stage.GetPrimAtPath(robot_prim_path):
        print("Spawning Unitree GO2 robot at prim path:", robot_prim_path)

        
        omni.kit.commands.execute(
            "CreateReference",
            usd_context=usd_context,
            path_to=robot_prim_path,
            asset_path=robot_asset_path
        )

        xform = UsdGeom.Xform.Get(stage, Sdf.Path(robot_prim_path))
        translate = Gf.Vec3d(0, 0, 0.29)
        physicsUtils.set_or_add_translate_op(xform, translate=translate)

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
        omni.kit.commands.execute(
            "ChangeTransform",
            path=robot_prim_path,
            value="translateZ:100.6383"  # Adjust the value so the feet touch the ground
        )

        
    else:
        print("Robot is already spawned at prim path:", robot_prim_path)
        omni.kit.commands.execute(
            "ChangeTransform",
            path=robot_prim_path,
            value="translateZ:100.6383"  # Adjust the value so the feet touch the ground
        )

class IMUStatePublisher(Node):
    def __init__(self, imu_xform_path):
        node_name = 'imu_publisher'

        # Initialize a temporary node to check for existing node names
        temp_node = Node('temp_node')
        node_names_and_namespaces = temp_node.get_node_names_and_namespaces()
        temp_node.destroy_node()

        # Extract just the node names from the tuples
        existing_node_names = [name for name, namespace in node_names_and_namespaces]

        if node_name in existing_node_names:
            raise RuntimeError(
                f"A node named '{node_name}' already exists. Please choose a different name.")

        # Proceed with node initialization
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Imu, 'unitree_go2/imu', 10)
        self.imu_xform_path = imu_xform_path

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"

        stage = get_current_stage()
        imu_prim = stage.GetPrimAtPath(self.imu_xform_path)
        if not imu_prim.IsValid():
            self.get_logger().error(
                f"IMU prim not found at path: {self.imu_xform_path}")
            return

        imu_xform = UsdGeom.Xformable(imu_prim)
        transform = imu_xform.ComputeLocalToWorldTransform(0)
        rotation = transform.ExtractRotation()
        quat = rotation.GetQuaternion()

        imu_msg.orientation.x = quat.GetImaginary()[0]
        imu_msg.orientation.y = quat.GetImaginary()[1]
        imu_msg.orientation.z = quat.GetImaginary()[2]
        imu_msg.orientation.w = quat.GetReal()

        angular_velocity_attr = imu_prim.GetAttribute("angularVelocity")
        linear_acceleration_attr = imu_prim.GetAttribute("linearAcceleration")

        if angular_velocity_attr and angular_velocity_attr.HasValue():
            angular_velocity = angular_velocity_attr.Get()
            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]
        else:
            self.get_logger().warn(
                "Angular velocity attribute not found or has no value.")

        if linear_acceleration_attr and linear_acceleration_attr.HasValue():
            linear_acceleration = linear_acceleration_attr.Get()
            imu_msg.linear_acceleration.x = linear_acceleration[0]
            imu_msg.linear_acceleration.y = linear_acceleration[1]
            imu_msg.linear_acceleration.z = linear_acceleration[2]
        else:
            self.get_logger().warn(
                "Linear acceleration attribute not found or has no value.")

        self.publisher_.publish(imu_msg)
        self.get_logger().info("Published IMU data.")

class JointStatePublisher(Node):
    def __init__(self, articulation, joint_names):
        node_name = 'joint_state_publisher'

        # Initialize a temporary node to check for existing node names
        temp_node = Node('temp_node')
        node_names_and_namespaces = temp_node.get_node_names_and_namespaces()
        temp_node.destroy_node()

        # Extract just the node names from the tuples
        existing_node_names = [name for name, _ in node_names_and_namespaces]

        if node_name in existing_node_names:
            raise RuntimeError(
                f"A node named '{node_name}' already exists. Please choose a different name.")

        # Proceed with node initialization
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(
            JointState, 'unitree_go2/joint_states', 10)
        self.articulation = articulation
        self.joint_names = joint_names

    def publish_joint_states(self):
        positions = self.articulation.get_joint_positions()
        if positions is None:
            self.get_logger().warn(
                "Articulation positions are None. Check initialization.")
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [float(p) for p in positions]

        self.publisher_.publish(joint_state_msg)
        self.get_logger().info(f"Published joint states: {joint_state_msg.position}")

def initialize_articulation_callback(event):
    global initialization_done
    if initialization_done:
        print("Articulation already initialized, skipping.")
        return

    if event.type == omni.timeline.TimelineEventType.CURRENT_TIME_TICKED.value:
        print("Timeline ticked, initializing articulation safely.")

        try:
            timeline.pause()
            articulation.initialize()
            print("Articulation initialized and joints set.")
            timeline.play()
        except Exception as e:
            print(f"Error during articulation initialization: {e}")
        finally:
            timeline.play()
        initialization_done = True
        timeline_sub.unsubscribe()
    
    # Assuming 'articulation' is your Articulation object
    joint_names = joints
    joint_positions = articulation.get_joint_positions()


    # Define the target position for the front left leg joint
    target_position = -52  # Example value in radians
    joint_index = joint_names.index('FL_thigh_joint')

    timeline.play()


def initialize_ros_nodes(imu_xform_path, articulation, joint_names):
    imu_publisher = IMUStatePublisher(imu_xform_path)
    joint_state_publisher = JointStatePublisher(articulation, joint_names)
    return imu_publisher, joint_state_publisher

def joint_wave_callback(step_size):
    # Read the current simulation time from the world's timeline.
    current_time = timeline.get_current_time()
    # Compute the wave value using a sine wave formula.
    wave_value = math.sin(2 * math.pi * frequency * current_time) * amplitude

    # Retrieve the current joint positions from the already-initialized articulation.
    joint_positions = articulation.get_joint_positions()
    if joint_positions is None:
        print("Warning: Joint positions not available.")
        return

    # For this example, update joint 0 with the computed wave value.
    joint_positions[0] = wave_value

    # Create an action containing the updated joint positions.
    action = ArticulationAction(joint_positions=joint_positions)
    try:
        articulation.apply_action(action)
    except Exception as e:
        print("Error applying joint action:", e)

# from concurrent.futures import ThreadPoolExecutor

# # Create a thread pool with a fixed number of worker threads.
# publish_executor = ThreadPoolExecutor(max_workers=4)
# publish_counter = 0
# callback_registered = False

# def combined_callback(step_size):
#     global publish_counter
#     publish_counter += 1

#     # Only submit tasks every 10th callback (adjust as needed)
#     if publish_counter % 20 == 0:
#         print("physics callback -> step_size:", step_size)
#         publish_executor.submit(imu_publisher.publish_imu_data)
#         publish_executor.submit(joint_state_publisher.publish_joint_states)



publish_queue = queue.Queue()
publish_counter = 0
n_publish_frequency = 30




    


# def publish_worker():
#     while not stop_event.is_set():
#         try:
#             # Wait until physics step data is available with a timeout
#             data = publish_queue.get(timeout=0.1)
#         except queue.Empty:
#             continue

#         try:
#             # Publish IMU data
#             imu_publisher.publish_imu_data()
#         except Exception as e:
#             print(f"Error publishing IMU data: {e}")

#         try:
#             # Publish joint states
#             joint_state_publisher.publish_joint_states()
#         except Exception as e:
#             print(f"Error publishing joint states: {e}")

# # Start the publishing worker in a separate thread
# publisher_thread = threading.Thread(target=publish_worker, daemon=True)
# publisher_thread.start()


setup_env()

# timeline.play()

SimulationContext.clear_instance()
simulation_context = SimulationContext()  # Example: 60 Hz
# world = World()
# world.initialize_physics()


simulation_context = SimulationContext()
async def task():
    await simulation_context.initialize_simulation_context_async()
    print("Simulation context initialized.")
    simulation_context.add_physics_callback("callback_physics", wave_callback)
    # simulation_context.set_physics_dt(1.0/10.0, 1)
    # simulation_context.get_physics_dt()

run_coroutine(task())




articulation = Articulation(robot_prim_path)


timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
    initialize_articulation_callback, 1
)

# Create a thread pool with a fixed number of worker threads.
publish_executor = ThreadPoolExecutor(max_workers=4)
publish_counter = 0

def ros2_callback(step_size):
    global publish_counter
    publish_counter += 1

    # Only submit tasks every 10th callback (adjust as needed)
    if publish_counter % 10 == 0:
        print("Physics callback -> step_size:", step_size)
        try:
            publish_executor.submit(imu_publisher.publish_imu_data)
        except Exception as e:
            print(f"Error submitting IMU data publishing task: {e}")
        try:
            publish_executor.submit(joint_state_publisher.publish_joint_states)
        except Exception as e:
            print(f"Error submitting joint states publishing task: {e}")


def wave_callback(step_size):
    

    # Select a random joint index
    joint_index = 9 # np.random.randint(len(joints))

    # Get the current joint positions
    joint_positions = articulation.get_joint_positions()

    if joint_positions is None:
        return

    print(f"Number of joint positions: {len(joint_positions)}")

    # Oscillate the joint position using a sine wave
    oscillation_amplitude = np.deg2rad(180)  # Convert 180 degrees to radians
    oscillation_frequency = 0.5  # Oscillations per second
    current_time = timeline.get_current_time()
    joint_positions[joint_index] = oscillation_amplitude * math.sin(2 * math.pi * oscillation_frequency * current_time)

    # Create an action containing the updated joint positions
    action = ArticulationAction(joint_positions=joint_positions)

    # Apply the action to the articulation
    articulation.apply_action(action)


if rclpy.utilities.ok():
    print("Shutting down previous ROS2 context...")
    rclpy.shutdown()

print("Initializing ROS2.")
rclpy.init()

imu_publisher, joint_state_publisher = initialize_ros_nodes(
    robot_imu_prim_path, articulation, joints)

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(imu_publisher)
executor.add_node(joint_state_publisher)

def main():
    # Initialize ROS2 and create publisher node
    try:

        # Initialize ROS2 if not already initialized
        if not rclpy.utilities.ok():
            print("Initializing ROS2.")
            rclpy.init()

        # Create a multi-threaded executor to run the node
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(imu_publisher)
        executor.add_node(joint_state_publisher)
        # Run executor in its own thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # Initialize physics *before* starting the timeline
        # world.initialize_physics()
        timeline.play()

        start_time = time.time()
        duration = 10  # seconds
        while not stop_event.is_set():  # Prevents thread from exiting
            if time.time() - start_time > duration:
                stop_event.set()  # Signal the main loop to stop

            # Call the combined callback function
            # combined_callback()

            time.sleep(2)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down ROS2.")
    finally:
        print("Shutting down ROS2.")
        rclpy.shutdown()
        print("ROS2 shut down successfully.")
        # timeline.stop()
        print("Timeline stopped.")
        # action = ArticulationAction(joint_positions=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.04, 1.04, 1.0, 1.0, 1.0]))
        # articulation.apply_action(action)


def set_joint_damping(joint_name, damping_value):
    # Construct the full prim path for the joint.
    joint_prim_path = f"{robot_prim_path}/{joint_name}"
    joint_prim = stage.GetPrimAtPath(joint_prim_path)
    if not joint_prim.IsValid():
        print(f"Joint prim not found: {joint_prim_path}")
        return

    # Apply a DriveAPI (for example, linear drive) to the joint prim
    driveAPI = UsdPhysics.DriveAPI.Apply(joint_prim, "linearDrive")
    driveAPI.CreateDampingAttr(damping_value)
    print(f"Set damping for {joint_name} to {damping_value}")

def set_all_joints_damping(damping_value):
    for joint in joints:
        set_joint_damping(joint, damping_value)

# Usage: set all joints to have a damping of 1000.0
set_all_joints_damping(1000.0)


main_thread = threading.Thread(target=main, daemon=True)
main_thread.start()
print("Main function is running in a separate thread.")

# action = ArticulationAction(joint_positions=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.04, 1.04, 1.0, 1.0, 1.0]))
# articulation.apply_action(action)