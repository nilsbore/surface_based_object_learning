import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from std_srvs.srv import Trigger



if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)
    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
#    rospy.wait_for_service('update_world_model')
    print("got it")

    print("waiting for pointcloud message")
    #cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    print("got a pointcloud")


    print("making proxy")
    #world_update = rospy.ServiceProxy('update_world_model',WorldUpdate)
    try:
        begin_obs = rospy.ServiceProxy('/begin_observations',Trigger)
        b = begin_obs()
        print("done")

        print("begin obs:")
        print(b)
    except rospy.ServiceException, e:
        print("ouch")
    #end_obs = rospy.ServiceProxy('/end_observations',Trigger)

    #print("end obs")
    #e = end_obs()
    #print(e)
    #print("done")

    #beginprint("executing service call")
    #world_update(input=cloud,waypoint="WayPoint32")
    print("done")
