#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion
import sensor_msgs.point_cloud2 as pc2
#import python_pcd
import tf
from recognition_srv_definitions.srv import *

import python_pcd
import PyKDL
import tf2_ros
from cluster_tracking_strategies import ClusterScore


class RecognitionResult:

    def __init__(self, label, confidence, cloud):
        self.confidence = confidence
        self.label = label
        self.cloud = cloud


class ObjectRecognitionManager:

    def transform_to_kdl(self, t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                     t.transform.rotation.z, t.transform.rotation.w),
                           PyKDL.Vector(t.transform.translation.x,
                                        t.transform.translation.y,
                                        t.transform.translation.z))

    def transform_cloud(self, cloud):
        t = self.listener.getLatestCommonTime("map", self.frame)
        self.listener.waitForTransform(
            "map", self.frame, t, rospy.Duration(5.0))
        t, r = self.listener.lookupTransform("map", self.frame, t)

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
            points_out.append([p_out[0], p_out[1], p_out[2], p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res

    def __init__(self):
        rospy.loginfo("-- Waiting for Object Recognition Service --")
        rospy.loginfo("-- 10 SECONDS --")
        self.setup_clean = False
        try:
            rospy.wait_for_service("/recognition_service/sv_recognition", 10)
            self.setup_clean = True
        except Exception, e:
            rospy.logwarn(
                "Could not get singleview recognition service, recognition will not be performed")

        rospy.loginfo("-- Got recognition service --")
        self.listener = tf.TransformListener()
        # let the listener grab a few frames of tf
        rospy.sleep(1)
        self.rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)

    def get_most_likely_label(self, cluster):
        # get the map bbox of cluster
        if(self.setup_clean is False):
            rospy.logwarn(
                "*** World_state_manager does not have recognition service")
            return

        bbox = cluster.bbox
        rospy.loginfo("-- Getting most likely label for cluster --")
        if(cluster.label):
            rospy.loginfo("-- Cluster already has a label--")
        else:
            rospy.loginfo("-- Cluster is as yet unlabelled --")

        # test against all the poits in all the result objects

        uk = RecognitionResult("unknown", 0.0, None)
        scores = {}
        scores[(uk, cluster)] = ClusterScore(uk, cluster, 0)
        for result in self.recog_results:
            cloud = self.transform_cloud(result.cloud)
            # translate cloud to map

            scores[(result, cluster)] = ClusterScore(result, cluster, 0)

            total = 0
            for p in pc2.read_points(cloud):
                point = [p[0], p[1], p[2]]
                total += 1
                if(bbox.contains_point(point)):
                    scores[(result, cluster)].score = scores[
                        (result, cluster)].score + 1
            scores[(result, cluster)].score = (float)(
                scores[(result, cluster)].score / total)

        # find the one that matches the best

        s_max = -1
        best = ClusterScore(uk, cluster, 0)

        if(cluster.label):
            best = cluster.label
            rospy.loginfo("setting best to existing label: " +
                          best.one.label + " at confidence: " + str(best.one.confidence))

        rospy.loginfo("LABEL \t \t \t MATCH SCORE")
        for s in scores:
            rospy.loginfo(str(scores[s].one.label) +
                          " \t \t \t " + str(scores[s].score))

            if(scores[s].one.confidence < best.one.confidence):
                continue

            if(scores[s].score > s_max):
                best = scores[s]
                s_max = scores[s].score

        return best.one.label, best.one.confidence

    def assign_labels(self, scene):
        rospy.loginfo("Assigning Labels")
        for cluster in scene.cluster_list:
            rospy.loginfo("Processing Cluster " + cluster.cluster_id)
            label, confidence = self.get_most_likely_label(cluster)
            cluster.label = label
            cluster.confidence = confidence

    def recognise_scene(self, input_cloud):
        if(self.setup_clean is False):
            rospy.logwarn(
                "*** World_state_manager does not have recognition service")
            return False
        rospy.loginfo("--- running object recognition ---")
        # run recogniser on input cloud
        self.frame = input_cloud.header.frame_id

        if(self.frame in "/pcd_cloud"):
            self.frame = "/head_xtion_depth_frame"

        rospy.loginfo("Input frame id: " + self.frame)

        tr_r = []

        rospy.loginfo("Running Recognition")
        try:
            response = self.rec_service(cloud=input_cloud)
        except Exception, e:
            rospy.logwarn("Recognition failed")

        #rospy.loginfo(response)

        self.recog_results = []

        for label, conf, cloud in zip(response.ids, response.confidence, response.models_cloud):
            r = RecognitionResult(label.data, conf, cloud)
            self.recog_results.append(r)
            rospy.loginfo("RESULT: " + str(r.label) + " at " + str(r.confidence) + " confidence")

        rospy.loginfo("--- recognition complete ---")
        return True


if __name__ == '__main__':
    rospy.init_node('recog_manager_test', anonymous=True)
#    r = ObjectRecognitionManager()
    print("done, testing")


    rospy.loginfo("-- Waiting for Object Recognition Service --")
    rospy.loginfo("-- 10 SECONDS --")
    try:
        rospy.wait_for_service("/recognition_service/sv_recognition", 10)
    except Exception, e:
        rospy.logwarn("Could not get singleview recognition service, recognition will not be performed")

    rospy.loginfo("-- Attaching recognition service --")

    rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)
    rospy.loginfo("-- Done! --")

    cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2,10)
    rospy.loginfo("-- Got Cloud --")




    rospy.spin()
