#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from soma_io.observation import Observation, TransformationStore
from soma_io.geometry import *
from soma_io.state import World, Object
from soma_io.observation import *
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion
import sensor_msgs.point_cloud2 as pc2
#import python_pcd
import tf
from recognition_srv_definitions.srv import *

import python_pcd
from cluster_tracker import SOMAClusterTracker

class RecognitionResult:
    def __init__(self,confidence,label,cloud):
        self.confidence = confidence
        self.label = label
        self.cloud = cloud

class ObjectRecognitionManager:

    def __init__(self):
        #rospy.init_node('recog_manager', anonymous = True)
        rospy.loginfo("--- created recognition manager ---")
        rospy.loginfo("-- Waiting for Recognition Service --")
        try:
            rospy.wait_for_service("/recognition_service/sv_recognition",10)
        except Exception,e:
            rospy.logwarn("Could not get singleview recognition service, recognition will not be performed")

        rospy.loginfo("-- Got recognition service --")
        self.listener = tf.TransformListener()
        # let the listener grab a few frames of tf
        rospy.sleep(1)
        self.rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)


    def recognise_scene(self,input_cloud):
        rospy.loginfo("--- running object recognition ---")
        # run recogniser on input cloud
        frame = input_cloud.header.frame_id

        if(frame in "/pcd_cloud"):
            frame = "/head_xtion_depth_optical_frame"
        rospy.loginfo("Input frame id: " + frame)

        tr_r = []

        try:
            rospy.loginfo("Looking for transform to map coordinates")
            t = self.listener.getLatestCommonTime("map", frame)
            self.listener.waitForTransform("map", frame, t, rospy.Duration(5.0))
            tr_r = self.listener.lookupTransform("map", frame, t)


            #tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
            #tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
        except Exception,e:
            rospy.logwarn("Failed to get transform to map co-ordinates")

        rospy.loginfo("Got transform successfully")
        rospy.loginfo("Running Recognition")

        response = self.rec_service(cloud=input_cloud,transform=tr_r[0])

        #self.recog_results = []

        #for label,conf,cloud in zip(response.ids,response.confidence,response.models_cloud):
        #    r = RecognitionResult(label,conf,cloud)
        #    self.recog_results.append(r)
        #    rospy.loginfo("RESULT: " + str(label) + " at " + str(conf) + " confidence")


        #rospy.loginfo("--- recognition complete ---")



if __name__ == '__main__':
    rospy.init_node('recog_manager_test', anonymous = True)
    r = ObjectRecognitionManager()
    print("done, testing")
    testcl = python_pcd.read_pcd("tests/cloud_00000012.pcd")
    r.recognise_scene(testcl[0])

    cluster_tracker = SOMAClusterTracker()
    cluster_tracker.add_unsegmented_scene(testcl)


    rospy.spin()
