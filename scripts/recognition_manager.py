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
import PyKDL
import tf2_ros
from cluster_tracking_strategies import ClusterScore

class RecognitionResult:
    def __init__(self,label,confidence,cloud):
        self.confidence = confidence
        self.label = label
        self.cloud = cloud

class ObjectRecognitionManager:

    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))

    def transform_cloud(self,cloud):
        t = self.listener.getLatestCommonTime("map", self.frame)
        self.listener.waitForTransform("map", self.frame, t, rospy.Duration(5.0))
        t,r = self.listener.lookupTransform("map", self.frame, t)

        tr = Transform()
        tr.translation.x = t[0]
        tr.translation.y = t[1]
        tr.translation.z = t[2]

        tr.rotation.x = r[0]
        tr.rotation.y = r[1]
        tr.rotation.z = r[2]
        tr.rotation.w = r[3]

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = "map"
        tr_s.child_frame_id = self.frame
        tr_s.transform = tr

        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res



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


    def get_most_likely(self,cluster):
        # get the map bbox of cluster
        bbox = cluster.bbox_local
        rospy.loginfo("-- Getting most likely label for cluster --")

        # test against all the poits in all the result objects
        scores = {}
        for result in self.recog_results:
            cloud = result.cloud
            scores[(result,cluster)] = ClusterScore(result,cluster,0)

            total = 0
            for p in pc2.read_points(cloud):
                point = [p[0],p[1],p[2]]
                total+=1
                if(bbox.contains_point(point)):
                    scores[(result,cluster)].score = scores[(result,cluster)].score+1
            scores[(result,cluster)].score = (float)(scores[(result,cluster)].score/total)

        # find the one that matches the best
        rospy.loginfo("LABEL \t \t \t CONFIDENCE")
        s_max = -1
        best = None
        for s in scores:
            rospy.loginfo(str(scores[s].one.label) + " \t \t \t " + str(scores[s].score))
            if(scores[s].score > s_max):
                best = s
                s_max = scores[s].score

        return best



    def recognise_scene(self,input_cloud):
        rospy.loginfo("--- running object recognition ---")
        # run recogniser on input cloud
        self.frame = input_cloud.header.frame_id

        if(self.frame in "/pcd_cloud"):
            self.frame = "/head_xtion_depth_frame"
        rospy.loginfo("Input frame id: " + self.frame)

        tr_r = []

        try:
            rospy.loginfo("Looking for transform to map coordinates")
            t = self.listener.getLatestCommonTime("map", self.frame)
            self.listener.waitForTransform("map", self.frame, t, rospy.Duration(5.0))
            tr_r = self.listener.lookupTransform("map", self.frame, t)


            #tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
            #tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
        except Exception,e:
            rospy.logwarn("Failed to get transform to map co-ordinates")

        rospy.loginfo("Got transform successfully")
        rospy.loginfo("Running Recognition")

        response = self.rec_service(cloud=input_cloud)

        self.recog_results = []

        for label,conf,cloud in zip(response.ids,response.confidence,response.models_cloud):
            r = RecognitionResult(label.data,conf,cloud)
            self.recog_results.append(r)
            rospy.loginfo("RESULT: " + str(r.label) + " at " + str(r.confidence) + " confidence")


        rospy.loginfo("--- recognition complete ---")



if __name__ == '__main__':
    rospy.init_node('recog_manager_test', anonymous = True)
    r = ObjectRecognitionManager()
    print("done, testing")
    testcl = python_pcd.read_pcd("tests/cloud_00000012.pcd")
    testcl = testcl[0]
    r.recognise_scene(testcl)

    cluster_tracker = SOMAClusterTracker()
    cluster_tracker.add_unsegmented_scene(testcl)


    for cluster in cluster_tracker.cur_scene.cluster_list:
        r.get_most_likely(cluster)


#    x = 0
    #y = 0
#    z = 0
    #c = 0

    #for cluster in cluster_tracker.cur_scene.cluster_list:
    #    print(cluster.map_centroid)
    #    print(cluster.local_centroid)
    ##    python_pcd.write_pcd("seg.pcd",cluster.segmented_pc_mapframe, overwrite=True)

    #for result in r.recog_results:
        #raw_cloud = pc2.read_points(result.cloud)
        #raw_cloud = r.transform_cloud(result.cloud)
        #python_pcd.write_pcd("rec.pcd",result.cloud, overwrite=True)

    #    for p in pc2.read_points(result.cloud):
    #        x+=p[0]
    #        y+=p[1]
    #        z+=p[2]
    #        c+=1

    #    x/=c
    #    y/=c
    #    z/=c
    #    print(x)
    #    print(y)
    #    print(z)
    #    print(c)

    rospy.spin()
