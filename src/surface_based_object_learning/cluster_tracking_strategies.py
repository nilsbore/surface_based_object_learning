#!/usr/bin/env python

import roslib
import rospy
import numpy as np
from view_registration import ViewAlignmentManager
import Queue as q
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.srv import *
import pcl

class BBox():
    """ Bounding box of an object with getter functions.
    """

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.x_min = x_min
        self.x_max = x_max

        self.y_min = y_min
        self.y_max = y_max

        self.z_min = z_min
        self.z_max = z_max

    def contains_point(self,point):
        if (self.x_max >= point[0] and self.x_min <= point[0] and self.y_max >= point[1] and self.y_min <= point[1] and self.z_max >= point[2] and self.z_min <= point[2]):
            return True
        return False

    def contains_pointstamped(self,point):
        if (self.x_max >= point.x and self.x_min <= point.x and self.y_max >= point.y and self.y_min <= point.y and self.z_max >= point.z and self.z_min <= point.z):
            return True
        return False

class ClusterTrackingStrategy:
    def __init__(self):
        self.talk = True
        rospy.loginfo("-- Using tracking strategy: ")


    def track(self,cur_scene,prev_scene,root_scene):
        rospy.loginfo("-- Performing Tracking")

class OctomapSimilarityTrackerStrategy(ClusterTrackingStrategy):

    def __init__(self):
        self.octo_similarity_service = rospy.ServiceProxy('/surface_based_object_learning/calculate_octree_overlap',CalculateOctreeOverlap)


    def track(self,cur_scene,prev_scene,root_scene):
        rospy.loginfo("OctomapSimilarityTracker")
        rospy.loginfo(""+str(len(cur_scene.segment_list)) + " clusters in this scene")
        rospy.loginfo(""+str(len(prev_scene.segment_list)) + " clusters in previous scene")
        if(cur_scene is None or prev_scene is None or root_scene is None):
            rospy.logerr("An input scene is null, not tracking")
            return
        if(self.octo_similarity_service is None):
            rospy.logerr("do not have octomap similarity service")
        # set all clusters to be unassigned
        cur_scene.reset_segment_assignments()
        prev_scene.reset_segment_assignments()

        print("calculating scores")
        for cur_seg in cur_scene.segment_list:
            if(cur_seg.assigned is True):
                continue
            # calculate the score of this segment against everything in the last scene
            best_score = 0
            best_segment = None
            for prev_seg in prev_scene.segment_list:
                if(prev_seg.assigned is True):
                    continue
                score = self.octo_similarity_service(cur_seg.segmented_pc_mapframe,prev_seg.segmented_pc_mapframe)
                # could threshold this further?
                rospy.loginfo("source: " + cur_seg.segment_id + " target: " + prev_seg.segment_id + " score:" + str(score))
                if(score < 25):
                    continue
                if(score > best_score):
                    best_score = score
                    best_segment = prev_seg
            cur_seg.assigned = True
            best_segment.assigned = True
            rospy.loginfo("\nbest cluster for " + cur_seg.segment_id + " found at cluster with id " + best_segment.segment_id + " and score " + str(best_score))
            cur_seg.segment_id = best_segment.segment_id



class ClusterScore:
    def __init__(self,one,two,score):
        self.one = one
        self.two = two
        self.score = score

    def __str__(self):
        return "c:" + str(self.one) + " p: " + str(self.two) + " score: " + str(self.score)


class NaiveClusterTrackingStrategy(ClusterTrackingStrategy):
    def track(self,cur_scene,prev_scene,root_scene):
        rospy.loginfo("NaiveClusterTrackingStrategy")
        rospy.loginfo(""+str(len(cur_scene.segment_list)) + " clusters in cur scene")
        rospy.loginfo(""+str(len(prev_scene.segment_list)) + " clusters in prev scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        queue = q.PriorityQueue()

        c_max = len(cur_scene.segment_list)
        num_assigned = 0

        for cc in cur_scene.segment_list:

            for cp in prev_scene.segment_list:
                dist = np.linalg.norm(cp.map_centroid-cc.map_centroid)
                idd = IndexDist(dist,cur_scene.segment_list.index(cc),prev_scene.segment_list.index(cp))
                queue.put(idd)

                #rospy.loginfo("cur cluster: " + str(cc.segment_id) + " prev cluster: " + str(cp.segment_id) + " dist: " + str(dist))
        while not queue.empty() and num_assigned < c_max:
            e = queue.get()

            if(cur_scene.segment_list[e.index_one].assigned == False):
                if(prev_scene.segment_list[e.index_two].assigned == False):

                    # the centroid of the new cluster
                    # is outside the bbox of the last
                #    if(cur_scene.segment_list[e.index_one].bbox.contains_point(prev_scene.segment_list[e.index_two].map_centroid)):
                        rospy.loginfo("new cluster's centroid in prev cluster")
                        cur_scene.segment_list[e.index_one].assigned = True
                        prev_scene.segment_list[e.index_two].assigned = True
                        rospy.loginfo("assigned cluster with index " + str(e.index_one) + " in cur frame, to cluster with index " + str(e.index_two) + " in prev frame, dist: " + str(e.dist))
                        cur_scene.segment_list[e.index_one].segment_id = prev_scene.segment_list[e.index_two].segment_id
                        rospy.loginfo("that cluster UUID is: " + str(cur_scene.segment_list[e.index_one].segment_id))
                        num_assigned+=1
                    #else:
                    #    rospy.loginfo("old cluster doesn't contain new centroid, dist: " + str(e.dist))
                    #    rospy.loginfo("cur centroid: " + str(cur_scene.segment_list[e.index_one].map_centroid))
                    #    bbox = prev_scene.segment_list[e.index_two].bbox
                    #    rospy.loginfo("bbox: [" + str(bbox.x_min) + "," + str(bbox.y_min) + "," +str(bbox.z_min) + "," + str(bbox.x_max) + "," + str(bbox.y_max) + ","+str(bbox.z_max)+"]")

        rospy.loginfo("assigned: " + str(num_assigned) + " clusters ")

class IndexDist:
    def __init__(self,dist,one,two):
        self.dist = dist
        self.index_one = one
        self.index_two = two

    def __cmp__(self, other):
        if self.dist < other.dist : return -1
        if self.dist == other.dist : return 0
        if self.dist > other.dist : return 1

    def __str__(self):
        return "c:" + str(self.index_one) + " p: " + str(self.index_two) + " dist: " + str(self.dist)
