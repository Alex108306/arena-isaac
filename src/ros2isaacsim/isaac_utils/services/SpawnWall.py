import omni
import numpy as np
from pxr import Gf, UsdGeom, Usd, Sdf, UsdShade
import math
from rclpy.qos import QoSProfile
from isaacsim_msgs.srv import SpawnWall
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.objects import DynamicCuboid, sphere, capsule, FixedCuboid
from omni.isaac.core import World

profile = QoSProfile(depth=2000)
def wall_spawner(request,response):
    #Get service attributes
    name = request.name
    world_path = request.world_path
    height = request.height
    
    start = np.append(np.array(request.start),height/2+0.1)
    end =  np.append(np.array(request.end),height/2+0.1)
    
    start_vec = Gf.Vec3d(*start)
    end_vec = Gf.Vec3d(*end)

    vector_ab = end - start 
    
    center = (start_vec + end_vec)/2
    length = np.linalg.norm(vector_ab[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])
    scale=Gf.Vec3f(*[length,0.05, height])

    #create wall
    stage = omni.usd.get_context().get_stage()
    world = World.instance()

    prim_path=f"{world_path}/Walls/{name}"

    world.scene.add(FixedCuboid(
    prim_path=prim_path,
    name = name,
    position=center,
    scale=scale,
    orientation = euler_angles_to_quat([0,0,angle]),
    color=np.array([0, 0, 0.02]),
    ))

    mdl_path = "/home/brainfucker/arena4_ws/src/arena/simulation-setup/entities/obstacles/static/canteen_wall/usd/Collected_SM_Wall_2m_198/Materials/MI_Props.mdl"
    mtl_path = "/World/Looks/WallMaterial"
    mtl = stage.GetPrimAtPath(mtl_path)
    if not (mtl and mtl.IsValid()):
        create_res = omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
            mtl_url=mdl_path,
            mtl_name='MI_Props',
            mtl_path=mtl_path)
    
    bind_res = omni.kit.commands.execute('BindMaterialCommand',
        prim_path=prim_path,
        material_path=mtl_path)

    

    response.ret = True
    return response

def spawn_wall(controller):
    service = controller.create_service(srv_type=SpawnWall, 
                        qos_profile = profile,
                        srv_name='isaac/spawn_wall', 
                        callback=wall_spawner)
    return service