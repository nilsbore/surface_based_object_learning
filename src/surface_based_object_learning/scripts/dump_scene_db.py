#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from soma_llsd_msgs.msg import *
import cv
import cv2
import os
import pickle

if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)

    bridge = CvBridge()
    store = MessageStoreProxy(database="somadata", collection="llsd_scene_store")
    scenes = store.query(Scene._type)
    targ = "scene_dump/"
    print("got some scenes." + str(len(scenes)) +" in fact! writing them to " + targ)
    if not os.path.exists(targ):
        os.makedirs(targ)

    scene_count = 0
    processed_episodes = []
    for sc in scenes:
        cur_scene = sc[0]
        print("processing: " + cur_scene.id)
        if not os.path.exists(targ+cur_scene.episode_id+"/"):
            os.makedirs(targ+cur_scene.episode_id+"/")
        if(cur_scene.episode_id not in processed_episodes):
            processed_episodes.append(cur_scene.episode_id)
        scene_count+=1
        pickle.dump(cur_scene,open(targ+cur_scene.episode_id+"/"+cur_scene.id+".p",'wb'))


    print("processed: " + str(scene_count) + " scenes and " + str(len(processed_episodes)) + " episodes")

    print("all done!")
