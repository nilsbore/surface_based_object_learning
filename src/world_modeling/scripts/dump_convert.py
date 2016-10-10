import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
from soma_io.observation import Observation, TransformationStore

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose
from soma_io.state import World, Object

import python_pcd
import pickle
from cluster_tracker import SOMAClusterTracker
from world_state_manager import WorldStateManager
import os

if __name__ == '__main__':
    #rospy.init_node('test_masks', anonymous = False)
    #print("getting soma service")
    #rospy.wait_for_service('soma2/query_db')
    #print("done")
    #soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
    #print("making query")

    base_dir = "/home/jxy/tsc_stuff/robot_logs/"
    bridge = CvBridge()

    # open root logs folder
    for episode in os.listdir(base_dir):
        # for each episode
        print("EPISODE: " + episode)
        for obj in os.listdir(base_dir+"/"+episode):
            print("\tOBJ: " + obj)
            for observation in os.listdir(base_dir+"/"+episode+"/"+obj):
                print("\t\tOBSERVATION: " + observation)
                targ = base_dir+"/"+episode+"/"+obj+"/"+observation+"/"

                rgb_img = pickle.load(open(targ+"image.p",'rb'))

                cv_img = bridge.imgmsg_to_cv2(rgb_img,"bgr8")

                cv2.imwrite(targ+"/"+"rgb.jpeg",cv_img)



    print("done")
