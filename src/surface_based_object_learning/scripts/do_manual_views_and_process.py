#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from surface_based_object_learning.srv import *
from std_srvs.srv import Trigger, TriggerResponse

if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
    #rospy.wait_for_service('update_world_model')
    print("got it")

    print("making proxy")
    begin_obs = rospy.ServiceProxy('/surface_based_object_learning/begin_observation_sequence',Trigger)
    end_obs = rospy.ServiceProxy('/surface_based_object_learning/end_observation_sequence',Trigger)
    process_scene = rospy.ServiceProxy('/surface_based_object_learning/process_scene',ProcessScene)

    print("done")

    invar = ""
    begin_obs()
    while(invar is not "q"):
        invar = raw_input('enter \'v\' to take view or \'q\' to quit: ')
        if(invar is 'q'):
            break
        print("getting cloud")
        cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
        process_scene(input=cloud,waypoint="WayPoint1",just_data_collection=False)
        print("done")
    print("ending sequence")
    end_obs()

    #rospy.spin()
