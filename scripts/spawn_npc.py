#!/usr/bin/env python3

import rospy
import carla
import random
from carla_ros_ad.srv import SpawnNPC

def spawn_n_npc(req):

    try:
        for i in range(req.n):
            vehicle_bp = random.choice(bp_library.filter('vehicle'))
            npc_veh = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
            npc_veh.set_autopilot(req.autopilot_state)
 
        rospy.loginfo("Service call succeeded!")
        return True
    except:
        rospy.logerr("Could not spawn NPC. Service call failed!")
        return False
    

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    
    rospy.init_node('spawn_npc')
    rospy.Service('/carla/my_spawn_npc', SpawnNPC, spawn_n_npc)
    rospy.spin()