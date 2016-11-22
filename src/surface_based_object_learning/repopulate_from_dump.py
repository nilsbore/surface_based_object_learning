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
from object_learning_core import *

if __name__ == '__main__':
    we = LearningCore("localhost","62345")

    for subdir, dirs, files in os.walk("scripts/scene_dump"):
        print("--episode--")
        for k in files:
            print("\t"+k)
            loaded = pickle.load( open(subdir+"/"+k, "rb" ) )
            observation_structure = {}
            observation_structure['rgb_image'] = loaded.rgb_img
            observation_structure['camera_info'] = loaded.camera_info
            observation_structure['scene_cloud'] = loaded.cloud
            observation_structure['robot_pose'] = loaded.robot_pose
            observation_structure['metadata'] = loaded.meta_data
            observation_structure['tf'] = loaded.transform
            observation_structure['depth_image'] = loaded.depth_img
            observation_structure['timestamp'] = loaded.timestamp

            we.process_scene(loaded.cloud,loaded.waypoint,observation_structure)
