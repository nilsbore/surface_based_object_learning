#!/usr/bin/env python

import roslib
import rospy
import numpy as np

import Queue as q

class ClusterTrackingStrategy:
    def __init__(self):
        self.talk = True
        if(self.talk): print("-- Using tracking strategy: ")


    def track(self,cur_scene,prev_scene):
        if(self.talk): print("-- Performing Tracking")


class VotingBasedClusterTrackingStrategy(ClusterTrackingStrategy):
    def track(self,cur_scene,prev_scene):
        if(self.talk): print("VotingBasedClusterTrackingStrategy")
        if(self.talk): print(""+str(len(cur_scene.cluster_list)) + " clusters in cur scene")
        if(self.talk): print(""+str(len(prev_scene.cluster_list)) + " clusters in prev scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        c_max = len(cur_scene.cluster_list)
        num_assigned = 0

        scores = {}

        for cur_cluster in cur_scene.cluster_list:
            for prev_cluster in prev_scene.cluster_list:
                # initalise score between these clusters to 0
                scores[(cur_cluster,prev_cluster)] = ClusterScore(cur_cluster,prev_cluster,0)
                for point in cur_cluster.data_world:
                    if(prev_cluster.bbox.contains_pointstamped(point.point)):
                        # increment the score if a point in the current cluster is in the bbox of the previous cluster
                        scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1
                        if(prev_cluster.outer_core_bbox.contains_pointstamped(point.point)):
                            print("point is in outer core!")
                            scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1

        print("raw scores")
        # normalise scores
        for s in scores:
            scores[s].score = (float)(scores[s].score)/(len(scores[s].one.data_world))
            print(str(scores[s].score))

        # assign cluster
        for i in scores:
            cur = scores[i].one
            best_score = 0
            best_cluster = None
            if(scores[i].one.assigned is False and scores[i].two.assigned is False):
                for j in scores:
                    if(scores[j].one == cur):
                        can = scores[j].two
                        if(scores[j].score >= scores[i].score):
                            if(scores[j].score > 0):
                                best_score = scores[j].score
                                best_cluster = can

                if(best_cluster != None):
                    if(self.talk): print("best score for: " + str(cur_scene.cluster_list.index(cur))  + " is: " + str(best_score) +" at best cluster: " + str(prev_scene.cluster_list.index(best_cluster)))
                    scores[i].one.assigned = True
                    scores[i].two.assigned = True
                    if(self.talk): print("assigned cluster with index " + str(cur_scene.cluster_list.index(cur))  + " in cur frame, to prev cluster with index " + str(prev_scene.cluster_list.index(best_cluster)))
                    cur.cluster_id = best_cluster.cluster_id
                    if(self.talk): print("that cluster UUID is: " + str(best_cluster.cluster_id))
                else:
                    if(self.talk): print("Couldn't find a good cluster for cluster: " + scores[i].one.cluster_id)
                    if(self.talk): print("(May be a brand new segment, or incomparable to previously seen segments)")

class ClusterScore:
    def __init__(self,one,two,score):
        self.one = one
        self.two = two
        self.score = score

    def __str__(self):
        return "c:" + str(self.one) + " p: " + str(self.two) + " score: " + str(self.score)


class NaiveClusterTrackingStrategy(ClusterTrackingStrategy):
    def track(self,cur_scene,prev_scene):
        if(self.talk): print("NaiveClusterTrackingStrategy")
        if(self.talk): print(""+str(len(cur_scene.cluster_list)) + " clusters in cur scene")
        if(self.talk): print(""+str(len(prev_scene.cluster_list)) + " clusters in prev scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        queue = q.PriorityQueue()

        c_max = len(cur_scene.cluster_list)
        num_assigned = 0

        for cc in cur_scene.cluster_list:

            for cp in prev_scene.cluster_list:
                dist = np.linalg.norm(cp.map_centroid-cc.map_centroid)
                idd = IndexDist(dist,cur_scene.cluster_list.index(cc),prev_scene.cluster_list.index(cp))
                queue.put(idd)

                #print("cur cluster: " + str(cc.cluster_id) + " prev cluster: " + str(cp.cluster_id) + " dist: " + str(dist))
        while not queue.empty() and num_assigned < c_max:
            e = queue.get()

            if(cur_scene.cluster_list[e.index_one].assigned == False):
                if(prev_scene.cluster_list[e.index_two].assigned == False):

                    # the centroid of the new cluster
                    # is outside the bbox of the last
                #    if(cur_scene.cluster_list[e.index_one].bbox.contains_point(prev_scene.cluster_list[e.index_two].map_centroid)):
                        if(self.talk): print("new cluster's centroid in prev cluster")
                        cur_scene.cluster_list[e.index_one].assigned = True
                        prev_scene.cluster_list[e.index_two].assigned = True
                        if(self.talk): print("assigned cluster with index " + str(e.index_one) + " in cur frame, to cluster with index " + str(e.index_two) + " in prev frame, dist: " + str(e.dist))
                        cur_scene.cluster_list[e.index_one].cluster_id = prev_scene.cluster_list[e.index_two].cluster_id
                        if(self.talk): print("that cluster UUID is: " + str(cur_scene.cluster_list[e.index_one].cluster_id))
                        num_assigned+=1
                    #else:
                    #    if(self.talk): print("old cluster doesn't contain new centroid, dist: " + str(e.dist))
                    #    if(self.talk): print("cur centroid: " + str(cur_scene.cluster_list[e.index_one].map_centroid))
                    #    bbox = prev_scene.cluster_list[e.index_two].bbox
                    #    if(self.talk): print("bbox: [" + str(bbox.x_min) + "," + str(bbox.y_min) + "," +str(bbox.z_min) + "," + str(bbox.x_max) + "," + str(bbox.y_max) + ","+str(bbox.z_max)+"]")

        if(self.talk): print("assigned: " + str(num_assigned) + " clusters ")

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
