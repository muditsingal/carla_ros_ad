#!/usr/bin/env python3

import rospy
import carla
from carla_ros_ad.srv import AutoPilot

def set_npc_autopilot(req):
    actor_list = world.get_actors()

    try:
        for actor in actor_list:
            if actor.type_id.startswith('vehicle.') and 'role_name' in actor.attributes and actor.attributes['role_name'] != 'ego_vehicle':
                actor.set_autopilot(req.state)    
        rospy.loginfo("Service call succeeded!")
        return True
    except:
        rospy.logerr("Could not perfom set npc autopilot to desired state. Service call failed!")
        return False
    

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    rospy.init_node('set_autopilot_node')
    rospy.Service('/carla/set_npc_autopilot', AutoPilot, set_npc_autopilot)
    rospy.spin()