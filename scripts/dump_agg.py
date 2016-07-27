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
import uuid

if __name__ == '__main__':
    #rospy.init_node('test_masks', anonymous = False)
    #print("getting soma service")
    #rospy.wait_for_service('soma2/query_db')
    #print("done")
    #soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
    #print("making query")

    base_dir = "/home/jxy/tsc_stuff/robot_logs_cutdown/"
    bridge = CvBridge()
    world_state_manager = WorldStateManager("localhost",62345)

    # open root logs folder
    episode_list = []
    for episode in os.listdir(base_dir):
        # for each episode

        print("EPISODE: " + episode)
        ep = []
        for obj in os.listdir(base_dir+"/"+episode):
            print("\tOBJ: " + obj)
            for observation in os.listdir(base_dir+"/"+episode+"/"+obj):
                #print("\t\tOBSERVATION: " + observation)
                target = base_dir+"/"+episode+"/"+obj+"/"+observation
                if(os.path.isfile(target+"/"+"image.p")):
                    print("\t\tOBSERVATION: " + observation)
                    out = {}
                    out['camera_info'] = pickle.load(open(target+"/"+"camera_info.p",'rb'))
                    out['robot_pose']  = pickle.load(open(target+"/"+"robot_pose.p",'rb'))
                    out['tf']  = pickle.load(open(target+"/"+"tf.p",'rb'))
                    out['cloud']  = pickle.load(open(target+"/"+"cloud.p",'rb'))
                    out['rgb_image']  = pickle.load(open(target+"/"+"image.p",'rb'))
                    out['data'] = pickle.load(open(target+"/"+"data.p",'rb'))
                #    python_pcd.write_pcd("views/"+str(uuid.uuid4())+".pcd",out['cloud'])
                    ep.append(out)

                else:
                    print("\t\tNO OBS HERE")
        episode_list.append(ep)

        #print("HAVE: " + str(len(ep)) + " VIEWS FOR THIS EPISODE")

    for episode in episode_list:
        print("EPISODE:")
        world_state_manager.begin_obs(None)
        vc = 0
        for view in episode:
            cl = view['cloud']
            print("VIEW:"+str(view['data']))
            world_state_manager.flush_observation(view)
            vc+=1

        world_state_manager.end_obs(None)
        print("\n")


    print("done")
