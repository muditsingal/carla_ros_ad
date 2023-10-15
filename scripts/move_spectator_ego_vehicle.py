#!/usr/bin/env python3

import rospy
import carla
import numpy as np
from std_srvs.srv import Empty
from carla_ros_ad.srv import set_spectator_to_ego_vehicle

def find_and_set_spectator(req):
    spectator = world.get_spectator()
    actor_list = world.get_actors()

    # Find the ego vehicle (assuming it's the first vehicle)
    ego_vehicle = None
    for actor in actor_list:
        if actor.type_id.startswith('vehicle.') and 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor
            break

    if ego_vehicle == None:
        rospy.logerr("Ego vehicle not found. Service call failed!")

    try:
        veh_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2)), ego_vehicle.get_transform().rotation)
        spectator.set_transform(veh_transform)
        rospy.loginfo("Service call succeeded!")
        return True
    except:
        rospy.logerr("Could not perfom transform operations. Service call failed!")
        return False
    

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    rospy.init_node('set_spectator_node')
    rospy.Service('/carla/set_spectator_ego_vehicle', set_spectator_to_ego_vehicle, find_and_set_spectator)
    rospy.spin()