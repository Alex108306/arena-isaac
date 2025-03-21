import numpy as np
import carb
from pedestrian.simulator.logic.people.person import Person
from rclpy.qos import QoSProfile
from isaacsim_msgs.srv import Pedestrian
from omni.isaac.core import World
import omni.anim.navigation.core as nav

profile = QoSProfile(depth=2000)
def pedestrian_spawn(request,response):
    #Get service attributes
    world = World.instance()
    people = request.people
    for person in people:
        if person.controller_stats == False:
            p = Person(world, person.stage_prefix, person.character_name, person.initial_pose, person.orientation)
            inav = nav.acquire_interface()
            navmesh = inav.get_navmesh()
            if navmesh:
                navmesh_path = navmesh.query_shortest_path(person.initial_pose, person.goal_pose)
                if navmesh_path:
                    path_points = navmesh_path.get_points()
                else:
                    carb.log_error(f"NavMesh could not query points")
            p.update_target_position(path_points, person.velocity)
        else:
            p = Person(world, person.stage_prefix, person.character_name, person.initial_pose, person.orientation, person.controller_name)

    response.ret = True
    return response

def spawn_ped(controller):
    service = controller.create_service(srv_type=Pedestrian, 
                        qos_profile = profile,
                        srv_name='isaac/spawn_pedestrian', 
                        callback=pedestrian_spawn)
    return service
