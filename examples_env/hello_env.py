import time
import threading
import omni.usd
from pxr import Sdf
import omni.isaac.dynamic_control as dc
import omni.kit.commands
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
import carb

from isaacsim.examples.interactive.base_sample import BaseSample


# Get the environment context
usd_context = omni.usd.get_context()
stage = usd_context.get_stage()
timeline = omni.timeline.get_timeline_interface()
stop_event = threading.Event()

# Define the prim path where the robot will be spawned and the asset path
robot_prim_path = "/World/UnitreeGo2"
robot_asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Unitree/Go2/go2.usd"

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()


        add_reference_to_stage(usd_path=robot_asset_path, prim_path=robot_prim_path)
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        unitreeGo2 = world.scene.add(Robot(prim_path=robot_prim_path, name="UnitreeGO2"))
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " + str(unitreeGo2.num_dof)) # prints None
        return

        async def setup_post_load(self):
            self._world = self.get_world()
            self._jetbot = self._world.scene.get_object("fancy_robot")
            # Print info about the jetbot after the first reset is called
            print("Num of degrees of freedom after first reset: " + str(self._jetbot.num_dof)) # prints 2
            print("Joint Positions after first reset: " + str(self._jetbot.get_joint_positions()))
            return
        
# Create a HelloWorld object and run the scene
hello_world = HelloWorld()
hello_world.setup_scene()

# Start the timeline to run the simulation
timeline.play()