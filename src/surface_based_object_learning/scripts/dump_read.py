mport roslib
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
    target_dir = "012bfcf5-1298-4433-8271-b87b0a3343a9"

    target = base_dir+target_dir
    print("opening target directory: " + target)
    out = {}
    out['camera_info'] = pickle.load(open(target+"/"+"camera_info.p",'rb'))
    out['robot_pose']  = pickle.load(open(target+"/"+"robot_pose.p",'rb'))
    out['tf']  = pickle.load(open(target+"/"+"tf.p",'rb'))
    out['cloud']  = pickle.load(open(target+"/"+"cloud.p",'rb'))
    python_pcd.write_pcd("/home/jxy/tsc_stuff/robot_logs/012bfcf5-1298-4433-8271-b87b0a3343a9/cloud.pcd",out['cloud'],overwrite=True)
    out['rgb_image']  = pickle.load(open(target+"/"+"image.p",'rb'))
    out['data'] = pickle.load(open(target+"/"+"data.p",'rb'))
    print(out['data'])

#    ct = SOMAClusterTracker()

#    ct.add_unsegmented_scene(out['cloud'],out)

    world_state_manager = WorldStateManager("localhost",62345)

    world_state_manager.flush_observation(out)
    #world_state_manager.place_in_view_store(out['cloud'],out)

    #rospy.spin()

    print("done")
