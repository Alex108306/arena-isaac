# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"headless": False}

simulation_app = SimulationApp(CONFIG)

import carb
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.core import SimulationContext
from pxr import Sdf, Gf

EXTENSIONS_PEOPLE = [
    'omni.anim.people', 
    'omni.anim.navigation.bundle', 
    'omni.anim.timeline',
    'omni.anim.graph.bundle', 
    'omni.anim.graph.core', 
    'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 
    'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 
    'omni.kit.scripting',
    'omni.graph.io',
    'omni.anim.curve.core',
    'omni.anim.navigation.core'
]

for ext_people in EXTENSIONS_PEOPLE:
    enable_extension(ext_people)

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
import omni.usd
omni.usd.get_context().new_stage()

import rclpy
import numpy as np
import omni.anim.navigation.core as nav
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.dynamic_control import _dynamic_control
import omni.anim.graph.core as ag

from isaacsim_msgs.srv import Pedestrian
from pedestrian.simulator.logic.people_manager import PeopleManager

#Import services
from isaac_utils.services import spawn_ped

world = World()
simulation_context = SimulationContext(stage_units_in_meters=1.0) #currently we use 1m for simulation.

stage_prefix = "/World"

prim = world.stage.DefinePrim(stage_prefix)

usd_asset = "/home/kuro/isaacsim4.2_ws/src/ros2isaacsim/world/House17_ver2_withdoor.usd"

success = prim.GetReferences().AddReference(usd_asset)

light_1 = prims.create_prim(
    "/World/Light_1",
    "DomeLight",
    position=np.array([1.0, 1.0, 1.0]),
    attributes={
        "inputs:texture:format": "latlong",
        "inputs:intensity": 1000.0,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)

# omni.timeline.get_timeline_interface().play()
simulation_app.update()

stage = omni.usd.get_context().get_stage()

# dc = _dynamic_control.acquire_dynamic_control_interface()
# # prim = stage.GetPrimAtPath("/World/House17_ver2/SlidingDoor/Doors")
# # matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
# # translate: Gf.Vec3d = matrix.ExtractTranslation()
# # print(translate)
# art = dc.get_articulation("/World/House17_ver2/SlidingDoor/Doors")

omni.kit.commands.execute("CreateNavMeshVolumeCommand",
    parent_prim_path=Sdf.Path("/World"),
    layer=stage.GetRootLayer()
)
simulation_app.update()

omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
            value=35.0)

omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
            value=True)
omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.people/navigation_settings/navmesh_enabled',
            value=True)

inav = nav.acquire_interface()
x = inav.start_navmesh_baking()
simulation_app.update()

#===================================controller====================================
# create controller node for isaacsim.
def create_controller(time=120):
    # init controller.
    controller = rclpy.create_node('controller')
    controller.create_publisher
    # init services.
    import_usd_service = spawn_ped(controller)
    return controller

# update the simulation.
def run(dc, art):
    simulation_app.update()

    people_list = PeopleManager.get_people_manager().people
    min_dist = 10.0
    if len(people_list) > 0:
        prim = stage.GetPrimAtPath("/World/House17_ver2/SlidingDoor/Doors")
        matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
        translate: Gf.Vec3d = matrix.ExtractTranslation()
        for person in people_list:
            position = people_list.get(person)
            distance_to_the_door = np.linalg.norm(position - translate)
            if min_dist > distance_to_the_door:
                min_dist = distance_to_the_door

    if simulation_context.is_playing():
        dc.wake_up_articulation(art)
        dof_ptr = dc.find_articulation_dof(art, "PrismaticJoint_R")
        if min_dist < 2.0:
            dc.set_dof_position_target(dof_ptr, 1.0)
        if min_dist > 2.0:
            dc.set_dof_position_target(dof_ptr, 0.0)
    # simulation_context.play()
#=================================================================================

#======================================main=======================================
def main(arg=None):
    rclpy.init()
    controller = create_controller()
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()
    dc = _dynamic_control.acquire_dynamic_control_interface()
    art = dc.get_articulation("/World/House17_ver2/SlidingDoor/Doors")
    while True:
        run(dc, art)
        rclpy.spin_once(controller, timeout_sec=0.0)

    controller.destroy_node()
    rclpy.shutdown()
    return
#=================================================================================
if __name__ == "__main__":
    main()