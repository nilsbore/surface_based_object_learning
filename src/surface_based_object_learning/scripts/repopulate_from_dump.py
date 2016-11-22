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

    for subdir, dirs, files in os.walk("scene_dump"):
        print("--episode--")
        for k in files:
            print("\t"+k)
            loaded = pickle.load( open(subdir+"/"+k, "rb" ) )
            
