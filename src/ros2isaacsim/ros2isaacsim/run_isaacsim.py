# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# Import dependencies.
import carb
import omni
import omni.graph.core as og
import usdrt.Sdf
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.prims import delete_prim,get_prim_at_path,set_prim_attribute_value,get_prim_attribute_value,get_prim_attribute_names
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
from pxr import Gf, Usd, UsdGeom
import numpy as np
import rclpy
from rclpy.node import Node
from isaacsim_msgs.msg import Euler, Quat, Env
from isaacsim_msgs.srv import ImportUsd, ImportUrdf, UrdfToUsd, DeletePrim, GetPrimAttributes, MovePrim
from sensor_msgs.msg import JointState

#======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
world = World()
extensions.enable_extension("omni.isaac.ros2_bridge")
simulation_app.update() #update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0) #currently we use 1m for simulation.

# Setting up URDF importer.
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True
import_config.convex_decomp = False
import_config.import_inertia_tensor = False
import_config.self_collision = False
import_config.fix_base = False
import_config.distance_scale = 1
import_config.make_default_prim = True
import_config.default_drive_type = (_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)
extension_path = _urdf.ImportConfig()

# list devices.
robots = []
environments = []
robot_positions = []
robot_rotations = []
environment_positions = []
environment_rotations = []

#================================================================================
#============================urdf converter service===============================
# URDF convert to usd (service) -> usd_path.
def urdf_to_usd(request, response):
    name = request.name
    urdf_path = request.urdf_path
    usd_path = f"robot_models/Arena_rosnav/User/{request.name}.usd"
    
    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        dest_path=usd_path,
        import_config=import_config,
        get_articulation_root=False,
    )
    
    response.usd_path = usd_path
    return response
    
# Urdf importer service callback.
def convert_urdf_to_usd(controller):
    service = controller.create_service(srv_type=UrdfToUsd, 
                        srv_name='urdf_to_usd', 
                        callback=urdf_to_usd)
    return service
#================================================================================
#========================publish environment information=========================
def publish_environemnt_information(node):
    msg = Env()
    msg.robots = robots
    msg.environments = environments
    msg.robot_positions = robot_positions
    msg.robot_rotations = robot_rotations
    msg.environment_positions = environment_positions
    msg.environment_rotations = environment_rotations
    node.publish(msg)

def create_publish_environment_information(controller):
    controller.create_publisher()
#================================================================================
#=========================multiple usd importer service==========================
## pass
#================================================================================

#=========================Get Prims attribute service============================
def get_prim_attributes(request,response):
    name = request.name
    prim_path = request.prim_path
    response.attributes_name = get_prim_attribute_names(prim_path)
    response.translate = get_prim_attribute_value(prim_path,attribute_name="xformOp:translate")
    return response

def get_prim_attr(controller):
    service = controller.create_service(srv_type=GetPrimAttributes, 
                        srv_name='get_prim_attributes', 
                        callback=get_prim_attributes)
    return service
#================================================================================

#============================Move Prims service================================
def prim_mover(request,response):
    name = request.name
    prim_path = request.prim_path
    target = request.target
    set_prim_attribute_value(prim_path,attribute_name="xformOp:translate", value=np.array(target))
    response.ret = True
    return response

def move_prim(controller):
    service = controller.create_service(srv_type=MovePrim, 
                        srv_name='move_prim', 
                        callback=prim_mover)
    return service
#================================================================================

#============================Delete Prims service================================
def prim_deleter(request,response):
    name = request.name
    prim_path = request.prim_path
    delete_prim(prim_path)
    response.ret = True
    return response

def _delete_prim(controller):
    service = controller.create_service(srv_type=DeletePrim, 
                        srv_name='_delete_prim', 
                        callback=prim_deleter)
    return service
#================================================================================

#============================usd importer service================================
# Usd importer (service) -> bool.
def usd_importer(request, response):
    name = request.name
    usd_path = request.usd_path
    prim_path = request.prim_path + "/" + name
    stage.add_reference_to_stage(usd_path, prim_path)

    response.ret = True
    if not request.control:
        environments.append(prim_path)
        return response 
    robots.append(prim_path)
    # create default graph.
    og.Controller.edit(
        # default graph name for robots.
        {"graph_path": f"/{name}"},
        {
            # create default nodes.
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubcribeJoinState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            # connect node inputs and outputs.
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubcribeJoinState.inputs:execIn"),
                ("SubcribeJoinState.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                ("ROS2Context.outputs:context", "SubcribeJoinState.inputs:context"),
                ("SubcribeJoinState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"), #config publisher and subcriber.
                ("SubcribeJoinState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubcribeJoinState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubcribeJoinState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
            # set default values for nodes.
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 1),
                ("PublishJointState.inputs:targetPrim", [prim_path + "/base_footprint"]),
                ("ArticulationController.inputs:targetPrim", [prim_path]),
                ("ArticulationController.inputs:robotPath", prim_path),
                ("SubcribeJoinState.inputs:topicName", f"{name}_command"),
                ("PublishJointState.inputs:topicName", f"{name}_states"),
            ]
        }
    )
    return response

# Usd importer service callback.
def import_usd(controller):
    service = controller.create_service(srv_type=ImportUsd, 
                        srv_name='import_usd', 
                        callback=usd_importer)
    return service
#=================================================================================

#===================================controller====================================
# create controller node for isaacsim.
def create_controller(time=120):
    # init controller.
    controller = rclpy.create_node('controller')
    controller.create_publisher
    # init services.
    import_usd_service = import_usd(controller)
    urdf_to_usd_service = convert_urdf_to_usd(controller)
    delete_prim_service = _delete_prim(controller)
    get_prim_attributes_service = get_prim_attr(controller)
    move_prim_service = move_prim(controller)
    ##
    return controller

# update the simulation.
def run():
    simulation_app.update()
    simulation_context.play()
#=================================================================================

#======================================main=======================================
def main(arg=None):
    rclpy.init()
    world.scene.add_default_ground_plane()
    controller = create_controller()
    while True:
        run()
        rclpy.spin_once(controller, timeout_sec=0.0)
    controller.destroy_node()
    rclpy.shutdown()
    return
#=================================================================================
if __name__ == "__main__":
    main()
