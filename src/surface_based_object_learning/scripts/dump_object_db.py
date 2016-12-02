#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from soma_llsd_msgs.msg import *
from soma_msgs.msg import SOMAObject
from soma_manager.srv import *
from soma_llsd.srv import *
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
import cv
import cv2
import os
import pickle
import python_pcd

if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)

    bridge = CvBridge()

    soma_query_service = rospy.ServiceProxy('/soma/query_objects',SOMAQueryObjs)
    segment_query_service = rospy.ServiceProxy('/soma_llsd/get_segment',GetSegment)
    query = SOMAQueryObjsRequest()
    query.query_type = 0
    query.objecttypes=['unknown']
    response = soma_query_service(query)

    bridge = CvBridge()


    for k in response.objects:
        print("getting: " + k.id)
        object_target_dir = "object_dump/"+k.id+"/"
        if not os.path.exists(object_target_dir):
            os.makedirs(object_target_dir)
        segment_req = segment_query_service(k.id)
        # insert into the time map
        print("getting seg imgs")
        for obs in segment_req.response.observations:
            rgb = obs.rgb_cropped
            cv_rgb_image = bridge.imgmsg_to_cv2(rgb)
            cv2.imwrite(object_target_dir+str(segment_req.response.observations.index(obs))+".png",cv_rgb_image)
            python_pcd.write_pcd(object_target_dir+str(segment_req.response.observations.index(obs))+".pcd",obs.map_cloud,overwrite=True)



    print("all done!")
