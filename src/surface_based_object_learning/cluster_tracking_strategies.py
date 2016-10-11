#!/usr/bin/env python

import roslib
import rospy
import numpy as np
from view_registration import ViewAlignmentManager
import Queue as q
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

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

class ViewAlignedVotingBasedClusterTrackingStrategy(ClusterTrackingStrategy):

    def get_points(self,cloud):
        points = []
        for p in pc2.read_points(cloud):
            points.append([p[0],p[1],p[2]])
        return points


    def calculate_overlap(self,a,b):
        dist = np.linalg.norm(a-b)
        if(dist < 0.01):
            return True
        return False


    def calculate_bbox(self,cluster):
        min_x=min_y=min_z= 99999
        max_x=max_y=max_z = -99999

        for point in pc2.read_points(cluster):
            x = point[0]
            y = point[1]
            z = point[2]

            if(x <= min_x):
                min_x = x
            if(x > max_x):
                max_x = x

            if(y <= min_y):
                min_y = y
            if(y > max_y):
                max_y = y

            if(z <= min_z):
                min_z = z
            if(z > max_z):
                max_z = z

        #print(min_x)
        #print(max_x)
        #print(min_y)
        #print(max_y)
        #print(min_z)
        #print(max_z)
        return BBox(min_x,max_x,min_y,max_y,min_z,max_z)




    def track(self,cur_scene,prev_scene,root_scene,view_alignment_manager):
        rospy.loginfo("ViewAlignedVotingBasedClusterTrackingStrategy")
        rospy.loginfo(""+str(len(cur_scene.cluster_list)) + " clusters in this scene")
        rospy.loginfo(""+str(len(prev_scene.cluster_list)) + " clusters in previous scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        # align cur_scene and prev_scene clouds with root_scene
        # gives us: transform. apply this to the cluster clouds
        # recalculate bbox and points from this
        aligned_clusters = view_alignment_manager.register_scenes(cur_scene,prev_scene,root_scene)
        rospy.loginfo("TRACKER: done aligning")


        c_max = len(cur_scene.cluster_list)
        num_assigned = 0
        use_core = False

        scores = {}

        for cur_cluster in cur_scene.cluster_list:
            for prev_cluster in prev_scene.cluster_list:
                # initalise score between these clusters to 0
                scores[(cur_cluster,prev_cluster)] = ClusterScore(cur_cluster,prev_cluster,0)

                prev_aligned = None

                for x in aligned_clusters[prev_scene.scene_id]:
                    if x[0] == prev_cluster.cluster_id:
                        #rospy.loginfo("TRACKER: got prev aligned")
                        prev_aligned = x[1]
                        break

                prev_aligned_bbox = self.calculate_bbox(prev_aligned)


                cur_aligned = None
                for x in aligned_clusters[cur_scene.scene_id]:
                    if x[0] == cur_cluster.cluster_id:
                        #rospy.loginfo("TRACKER: got cur aligned")
                        cur_aligned = x[1]
                        break

                #rospy.loginfo("TRACKER: getting points")
                cur_aligned_points = self.get_points(cur_aligned)
                prev_aligned_points = self.get_points(prev_aligned)
                cur_aligned_bbox = self.calculate_bbox(cur_aligned)

            #    for a in cur_aligned_points:
            #        a_p = np.array((a[0],a[1],a[2]))
            #        for b in prev_aligned_points:
            #            b_p = np.array((b[0],b[1],b[2]))
            #            if(self.calculate_overlap(a_p,b_p)):
            #                scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1
                for point in cur_aligned_points:
                    if(prev_aligned_bbox.contains_point(point)):
                        scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1



        rospy.loginfo("raw scores")
        # normalise scores
        for s in scores:
            scores[s].score = (float)(scores[s].score)/(len(scores[s].one.data_world))
            rospy.loginfo(str(scores[s].score))

        # assign cluster
        assigned = []
        for i in scores:
            cur = scores[i].one
            best_score = 0
            best_cluster = None
            if(cur.cluster_id not in assigned):
                for j in scores:
                    if(scores[j].one.cluster_id in cur.cluster_id):
                        can = scores[j].two
                        if(can.cluster_id not in assigned):
                            rospy.loginfo(str(cur_scene.cluster_list.index(cur)) +" \t \t " + str(prev_scene.cluster_list.index(can)) +" \t \t " + str(scores[j].score))
                            if(scores[j].score > best_score):
                                best_score = scores[j].score
                                best_cluster = can

                if(best_cluster != None):
                    rospy.loginfo("best score for: " + str(cur_scene.cluster_list.index(cur))  + " is: " + str(best_score) +" at best cluster: " + str(prev_scene.cluster_list.index(best_cluster)))
                    assigned.append(cur.cluster_id)
                    assigned.append(best_cluster.cluster_id)
                    rospy.loginfo("assigned cluster with index " + str(cur_scene.cluster_list.index(cur))  + " in cur frame, to prev cluster with index " + str(prev_scene.cluster_list.index(best_cluster)))
                    cur.cluster_id = best_cluster.cluster_id
                    rospy.loginfo("that cluster UUID is: " + str(best_cluster.cluster_id))
                else:
                    rospy.loginfo("Couldn't find a good cluster for cluster: " + scores[i].one.cluster_id + "(May be a brand new segment, or incomparable to previously seen segments)")
            else:
                rospy.loginfo("Skipping assignment of " + cur.cluster_id + " as it's already been assigned")

class VoxelViewAlignedVotingBasedClusterTrackingStrategy(ClusterTrackingStrategy):


    def track(self,cur_scene,prev_scene,root_scene,view_alignment_manager):
        rospy.loginfo("VoxelViewAlignedVotingBasedClusterTrackingStrategy")
        rospy.loginfo(""+str(len(cur_scene.cluster_list)) + " clusters in this scene")
        rospy.loginfo(""+str(len(prev_scene.cluster_list)) + " clusters in previous scene")
        if(cur_scene is None or prev_scene is None or root_scene is None):
            rospy.logerr("An input scene is null, not tracking")
            return

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        # align cur_scene and prev_scene clouds with root_scene
        # gives us: transform. apply this to the cluster clouds
        # recalculate bbox and points from this
        aligned_clusters = view_alignment_manager.register_scenes(cur_scene,prev_scene,root_scene)
        rospy.loginfo("TRACKER: done aligning")


        c_max = len(cur_scene.cluster_list)
        num_assigned = 0
        use_core = False
        octree_res = 0.3
        scores = {}

        for cur_cluster in cur_scene.cluster_list:

            cur_aligned = None

            for x in aligned_clusters[cur_scene.scene_id]:
                if x[0] == cur_cluster.cluster_id:
                    cur_aligned = x[1]
                    break

            cur_cld = pcl.PointCloud()
            pts = []
            for point in pc2.read_points(cur_aligned):
                pts.append([point[0],point[1],point[2]])

            if not pts:
                continue

            pts = np.array(pts,dtype=np.float32)
            cur_cld.from_array(pts)
            cur_octree = cur_cld.make_octree(octree_res)


            cur_octree.add_points_from_input_cloud()

            cur_occupied = cur_octree.get_occupied_voxel_centers()

            for prev_cluster in prev_scene.cluster_list:
                prev_aligned = None
                for x in aligned_clusters[prev_scene.scene_id]:
                    if x[0] == prev_cluster.cluster_id:
                        prev_aligned = x[1]
                        break


                scores[(cur_cluster,prev_cluster)] = ClusterScore(cur_cluster,prev_cluster,0)
                pre_cld = pcl.PointCloud()
                pts = []
                for point in pc2.read_points(prev_aligned):
                    pts.append([point[0],point[1],point[2]])
                pts = np.array(pts,dtype=np.float32)

                for vc in pts:
                    if cur_octree.is_voxel_occupied_at_point(np.array(vc,dtype=np.float32)):
                        scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1

                if(pts.size > 0):
                    scores[(cur_cluster,prev_cluster)].score = (float)(scores[(cur_cluster,prev_cluster)].score)/(len(pts))
                else:
                    scores[(cur_cluster,prev_cluster)].score = 0


        # assign cluster
        assigned = []
        for i in scores:
            cur = scores[i].one
            best_score = 0
            best_cluster = None
            if(cur.cluster_id not in assigned):
                for j in scores:
                    if(scores[j].one.cluster_id in cur.cluster_id):
                        can = scores[j].two
                        if(can.cluster_id not in assigned):
                            rospy.loginfo(str(cur_scene.cluster_list.index(cur)) +" \t \t " + str(prev_scene.cluster_list.index(can)) +" \t \t " + str(scores[j].score))
                            if(scores[j].score > best_score):
                                best_score = scores[j].score
                                best_cluster = can

                if(best_cluster != None):
                    rospy.loginfo("best score for: " + str(cur_scene.cluster_list.index(cur))  + " is: " + str(best_score) +" at best cluster: " + str(prev_scene.cluster_list.index(best_cluster)))
                    assigned.append(cur.cluster_id)
                    assigned.append(best_cluster.cluster_id)
                    rospy.loginfo("assigned cluster with index " + str(cur_scene.cluster_list.index(cur))  + " in cur frame, to prev cluster with index " + str(prev_scene.cluster_list.index(best_cluster)))
                    cur.cluster_id = best_cluster.cluster_id
                    rospy.loginfo("that cluster UUID is: " + str(best_cluster.cluster_id))
                else:
                    rospy.loginfo("Couldn't find a good cluster for cluster: " + scores[i].one.cluster_id + "(May be a brand new segment, or incomparable to previously seen segments)")
            else:
                rospy.loginfo("Skipping assignment of " + cur.cluster_id + " as it's already been assigned")



class VoxelVotingBasedClusterTrackingStrategy(ClusterTrackingStrategy):
    def track(self,cur_scene,prev_scene):
        rospy.loginfo("VoxelVotingBasedClusterTrackingStrategy")
        rospy.loginfo(""+str(len(cur_scene.cluster_list)) + " clusters in this scene")
        rospy.loginfo(""+str(len(prev_scene.cluster_list)) + " clusters in previous scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        # align cur_scene and prev_scene clouds with root_scene
        # gives us: transform. apply this to the cluster clouds
        # recalculate bbox and points from this

        c_max = len(cur_scene.cluster_list)
        num_assigned = 0
        scores = {}
        octree_res = 0.01

        for cur_cluster in cur_scene.cluster_list:
            cur_cld = pcl.PointCloud()
            pts = []
            for point in pc2.read_points(cur_cluster.segmented_pc_mapframe):
                pts.append([point[0],point[1],point[2]])
            pts = np.array(pts,dtype=np.float32)
            cur_cld.from_array(pts)
            cur_octree = cur_cld.make_octree(octree_res)
            cur_octree.add_points_from_input_cloud()

            cur_occupied = cur_octree.get_occupied_voxel_centers()

            for prev_cluster in prev_scene.cluster_list:
                scores[(cur_cluster,prev_cluster)] = ClusterScore(cur_cluster,prev_cluster,0)
                pre_cld = pcl.PointCloud()
                pts = []
                for point in pc2.read_points(prev_cluster.segmented_pc_mapframe):
                    pts.append([point[0],point[1],point[2]])
                pts = np.array(pts,dtype=np.float32)
                pre_cld.from_array(pts)
                pre_octree = pre_cld.make_octree(octree_res)
                pre_octree.add_points_from_input_cloud()

                pre_occupied = pre_octree.get_occupied_voxel_centers()

                for vc in pre_occupied:
                    if cur_octree.is_voxel_occupied_at_point(np.array(vc,dtype=np.float32)):
                        scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1


                scores[(cur_cluster,prev_cluster)].score = (float)(scores[(cur_cluster,prev_cluster)].score)/(len(pre_occupied))

        assigned = []
        for i in scores:
            cur = scores[i].one
            best_score = 0
            best_cluster = None
            if(cur.cluster_id not in assigned):
                for j in scores:
                    if(scores[j].one.cluster_id in cur.cluster_id):
                        can = scores[j].two
                        if(can.cluster_id not in assigned):
                            rospy.loginfo(str(cur_scene.cluster_list.index(cur)) +" \t \t " + str(prev_scene.cluster_list.index(can)) +" \t \t " + str(scores[j].score))
                            if(scores[j].score >= scores[i].score):
                                if(scores[j].score > 0):
                                    best_score = scores[j].score
                                    best_cluster = can

                if(best_cluster != None):
                    rospy.loginfo("best score for: " + str(cur_scene.cluster_list.index(cur))  + " is: " + str(best_score) +" at best cluster: " + str(prev_scene.cluster_list.index(best_cluster)))
                    assigned.append(cur.cluster_id)
                    assigned.append(best_cluster.cluster_id)
                    rospy.loginfo("assigned cluster with index " + str(cur_scene.cluster_list.index(cur))  + " in cur frame, to prev cluster with index " + str(prev_scene.cluster_list.index(best_cluster)))
                    cur.cluster_id = best_cluster.cluster_id
                    rospy.loginfo("that cluster UUID is: " + str(best_cluster.cluster_id))
                else:
                    rospy.loginfo("Couldn't find a good cluster for cluster: " + scores[i].one.cluster_id + "(May be a brand new segment, or incomparable to previously seen segments)")



class VotingBasedClusterTrackingStrategy(ClusterTrackingStrategy):
    def track(self,cur_scene,prev_scene,root_scene):
        rospy.loginfo("VotingBasedClusterTrackingStrategy")
        rospy.loginfo(""+str(len(cur_scene.cluster_list)) + " clusters in this scene")
        rospy.loginfo(""+str(len(prev_scene.cluster_list)) + " clusters in previous scene")

        # set all clusters to be unassigned
        cur_scene.reset_cluster_assignments()
        prev_scene.reset_cluster_assignments()

        # align cur_scene and prev_scene clouds with root_scene
        # gives us: transform. apply this to the cluster clouds
        # recalculate bbox and points from this

        c_max = len(cur_scene.cluster_list)
        num_assigned = 0
        use_core = False

        scores = {}

        for cur_cluster in cur_scene.cluster_list:
            for prev_cluster in prev_scene.cluster_list:
                # initalise score between these clusters to 0
                scores[(cur_cluster,prev_cluster)] = ClusterScore(cur_cluster,prev_cluster,0)
                for point in cur_cluster.data_world:
                    if(prev_cluster.bbox.contains_pointstamped(point.point)):
                        # increment the score if a point in the current cluster is in the bbox of the previous cluster
                        scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1
                        if(use_core):
                            if(prev_cluster.outer_core_bbox.contains_pointstamped(point.point)):
                                #rospy.loginfo("point is in outer core!")
                                scores[(cur_cluster,prev_cluster)].score = scores[(cur_cluster,prev_cluster)].score+1

        rospy.loginfo("raw scores")
        # normalise scores
        for s in scores:
            scores[s].score = (float)(scores[s].score)/(len(scores[s].one.data_world))
            rospy.loginfo(str(scores[s].score))

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
                    rospy.loginfo("best score for: " + str(cur_scene.cluster_list.index(cur))  + " is: " + str(best_score) +" at best cluster: " + str(prev_scene.cluster_list.index(best_cluster)))
                    scores[i].one.assigned = True
                    scores[i].two.assigned = True
                    rospy.loginfo("assigned cluster with index " + str(cur_scene.cluster_list.index(cur))  + " in cur frame, to prev cluster with index " + str(prev_scene.cluster_list.index(best_cluster)))
                    cur.cluster_id = best_cluster.cluster_id
                    rospy.loginfo("that cluster UUID is: " + str(best_cluster.cluster_id))
                else:
                    rospy.loginfo("Couldn't find a good cluster for cluster: " + scores[i].one.cluster_id + "(May be a brand new segment, or incomparable to previously seen segments)")

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
        rospy.loginfo(""+str(len(cur_scene.cluster_list)) + " clusters in cur scene")
        rospy.loginfo(""+str(len(prev_scene.cluster_list)) + " clusters in prev scene")

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

                #rospy.loginfo("cur cluster: " + str(cc.cluster_id) + " prev cluster: " + str(cp.cluster_id) + " dist: " + str(dist))
        while not queue.empty() and num_assigned < c_max:
            e = queue.get()

            if(cur_scene.cluster_list[e.index_one].assigned == False):
                if(prev_scene.cluster_list[e.index_two].assigned == False):

                    # the centroid of the new cluster
                    # is outside the bbox of the last
                #    if(cur_scene.cluster_list[e.index_one].bbox.contains_point(prev_scene.cluster_list[e.index_two].map_centroid)):
                        rospy.loginfo("new cluster's centroid in prev cluster")
                        cur_scene.cluster_list[e.index_one].assigned = True
                        prev_scene.cluster_list[e.index_two].assigned = True
                        rospy.loginfo("assigned cluster with index " + str(e.index_one) + " in cur frame, to cluster with index " + str(e.index_two) + " in prev frame, dist: " + str(e.dist))
                        cur_scene.cluster_list[e.index_one].cluster_id = prev_scene.cluster_list[e.index_two].cluster_id
                        rospy.loginfo("that cluster UUID is: " + str(cur_scene.cluster_list[e.index_one].cluster_id))
                        num_assigned+=1
                    #else:
                    #    rospy.loginfo("old cluster doesn't contain new centroid, dist: " + str(e.dist))
                    #    rospy.loginfo("cur centroid: " + str(cur_scene.cluster_list[e.index_one].map_centroid))
                    #    bbox = prev_scene.cluster_list[e.index_two].bbox
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
