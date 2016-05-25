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


class ObjectRecognitionManager:

    def __init__(self):
        rospy.loginfo("--- created recognition manager ---")

    def recognise_scene(self,input_cloud):
        rospy.loginfo("--- running object recognition")

        # run recogniser on input cloud

        labels = ["a","b","c"]
        confidences = [0.4,0.2,0.4]
        clouds = []

        return labels,confidences,clouds


if __name__ == '__main__':
    rospy.loginfo("hi")
