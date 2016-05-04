#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv
import cv2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from segmentation_srv_definitions.srv import *
import numpy as np
from geometry_msgs.msg import PointStamped
import tf
import PyKDL
import tf2_ros
import geometry_msgs.msg
from cluster_tracking_strategies import NaiveClusterTrackingStrategy, VotingBasedClusterTrackingStrategy
from tf import transformations
import uuid
import random
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import base64
talk = False

class BBox():
    """ Bounding box of an object with getter functions.
    """
    def __init__(self, bbox):
        # Calc x_min and x_max for obj1
        x_sorted = sorted(bbox, key=itemgetter(0))
        self.x_min = x_sorted[0][0]
        self.x_max = x_sorted[7][0]

        # Calc y_min and y_max for obj
        y_sorted = sorted(bbox, key=itemgetter(1))
        self.y_min = y_sorted[0][1]
        self.y_max = y_sorted[7][1]

        # Calc z_min and z_max for obj
        z_sorted = sorted(bbox, key=itemgetter(2))
        self.z_min = z_sorted[0][2]
        self.z_max = z_sorted[7][2]

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.x_min = x_min
        self.x_max = x_max

        self.y_min = y_min
        self.y_max = y_max

        self.z_min = z_min
        self.z_max = z_max

    def get_x_min(self):
        return self.x_min

    def get_x_max(self):
        return self.x_max


    def get_y_min(self):
        return self.y_min

    def get_y_max(self):
        return self.y_max

    def get_z_min(self):
        return self.z_min

    def get_z_max(self):
        return self.z_max

    def contains_point(self,point):
        if (self.x_max >= point[0] and self.x_min <= point[0] and self.y_max >= point[1] and self.y_min <= point[1] and self.z_max >= point[2] and self.z_min <= point[2]):
            return True
        return False

    def contains_pointstamped(self,point):
        if (self.x_max >= point.x and self.x_min <= point.x and self.y_max >= point.y and self.y_min <= point.y and self.z_max >= point.z and self.z_min <= point.z):
            return True
        return False

    def is_colliding(self,ext_bb):
        x_min = self.get_x_min()
        x_max = self.get_x_max()
        y_min = self.get_y_min()
        y_max = self.get_y_max()

        e_x_min = ext_bb.get_x_min()
        e_x_max = ext_bb.get_x_max()
        e_y_min = ext_bb.get_y_min()
        e_y_max = ext_bb.get_y_max()

        if (x_max >= e_x_min and x_min <= e_x_max and y_max >= e_y_min and y_min <= e_y_max):
            return True
        return False


class SegmentedCluster:
    def __init__(self,idc,cloud):
        self.data = []
        self.cloud = cloud
        self.data_world = []
        self.cluster_id = idc
        self.map_centroid = None
        self.local_centroid = None
        self.bbox = None
        self.assigned = False

    def reset_assignment(self):
        self.assigned = False

class SegmentedScene:

    def reset_cluster_assignments(self):
        for cluster in self.cluster_list:
            cluster.reset_assignment()

    def contains_cluster_id(self,id):
        for cluster in self.cluster_list:
            if(cluster.cluster_id == id):
                return True
        return False

    def calculate_centroid(self,points):
        x = 0
        y = 0
        z = 0
        num = len(points)
        for point in points:
            x += point[0]
            y += point[1]
            z += point[2]

        x /= num
        y /= num
        z /= num

        return [x,y,z]


    def calculate_2d_image_centroid(self,centroid):
        model = image_geometry.PinholeCameraModel()
        print("waiting for camera")
        camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=3.0)
        model.fromCameraInfo(camera_msg)
        px = model.project3dToPixel((centroid[0], centroid[1], centroid[2]))
        return px



    def __init__(self,indices,cloud,listener):
        if(talk): print("\nthis cloud has " + str(len(indices.clusters_indices)) + " clusters")
        self.num_clusters = len(indices.clusters_indices)
        self.cloud = cloud

        print("transforming point cloud")
        translation,rotation = listener.lookupTransform("/map", "/head_xtion_rgb_optical_frame", rospy.Time())

        print("got it")

        self.raw_cloud = pc2.read_points(cloud)
        int_data = list(self.raw_cloud)

        self.cluster_list = []

        #
        print("getting image of scene")
        scene_img = rospy.wait_for_message("/head_xtion/rgb/image_rect_color",  Image, timeout=15.0)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(scene_img, desired_encoding="bgr8")
        print("got it")

        if(talk): print("loading clusters")
        for root_cluster in indices.clusters_indices:
            if(talk): print("--- CLUSTER ----")

            #cid = [random.choice("ABCDEFGHIGJSDHS903R4IDFA34RAFDAFDLLAFD") for _ in range(4)] # this call makes me very nervous
            #cid = ''.join(cid)
            cid = str(uuid.uuid4()) # str so we can later link it to a soma2 object, which indexes by string
            if(talk): print("randomly assigned temporary cid: " + str(cid))
            cur_cluster = SegmentedCluster(cid,root_cluster)
            #raw = []
            for idx in root_cluster.data:
                cur_cluster.data.append(int_data[idx])

            #if(talk): print("i added: " + str(len(cur_cluster.data)) + " points to a cluster")

            # make some space for calculating the world centroid of the cluster
            x = 0
            y = 0
            z = 0

            # make some space for calculating the local centroid of the cluster
            x_local = 0
            y_local = 0
            z_local = 0

            # initalise some extreme values for finding the dimensions of a
            # bounding rectangle around the object
            min_x=min_y=min_z=99999
            max_x=max_y=max_z = -99999
            local_min_x=local_min_y=local_min_z = 99999
            local_max_x=local_max_y=local_max_z = -99999

            #translation,rotation = listener.lookupTransform("/map", "/head_xtion_rgb_optical_frame", rospy.Time())
            trans_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

            #if(talk): print("got transform: ")
            #if(talk): print(trans_cache[0])

            raw = []

            model = image_geometry.PinholeCameraModel()
            print("waiting for camera")
            camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=3.0)
            model.fromCameraInfo(camera_msg)

            rgb_min_x = 90000
            rgb_max_x = 0

            rgb_min_y = 90000
            rgb_max_y = 0

            for points in cur_cluster.data:
                # store the roxe world transformed point too
                pt_s = PointStamped()
                pt_s.header = "/map"
                pt_s.point.x = points[0]
                pt_s.point.y = points[1]
                pt_s.point.z = points[2]

                # get the rgb pos of the point
                rgb_point = model.project3dToPixel((pt_s.point.x, pt_s.point.y, pt_s.point.z))
                #print("rgb point: " + str(rgb_point))

                rgb_x = rgb_point[0]
                rgb_y = rgb_point[1]

                if(rgb_x < rgb_min_x):
                    rgb_min_x = rgb_x

                if(rgb_y < rgb_min_y):
                    rgb_min_y = rgb_y

                if(rgb_x > rgb_max_x):
                    rgb_max_x = rgb_x

                if(rgb_y > rgb_max_y):
                    rgb_max_y = rgb_y


                x_local += pt_s.point.x
                y_local += pt_s.point.y
                z_local += pt_s.point.z

                if(pt_s.point.x > local_max_x):
                    local_max_x = pt_s.point.x

                if(pt_s.point.y > local_max_y):
                    local_max_y = pt_s.point.y

                if(pt_s.point.z > local_max_z):
                    local_max_z = pt_s.point.z

                if(pt_s.point.x < local_min_x):
                    local_min_x = pt_s.point.x

                if(pt_s.point.y < local_min_y):
                    local_min_y = pt_s.point.y

                if(pt_s.point.z < local_min_z):
                    local_min_z = pt_s.point.z

                xyz = tuple(np.dot(trans_matrix, np.array([pt_s.point.x, pt_s.point.y, pt_s.point.z, 1.0])))[:3]
                x += xyz[0]
                y += xyz[1]
                z += xyz[2]

                # transform to map co-ordinates
                pt_s.point = geometry_msgs.msg.Point(*xyz)

                color_data = points[3]
                
                raw.append((pt_s.point.x,pt_s.point.y,pt_s.point.z,color_data))

                cur_cluster.data_world.append(pt_s)

                if(pt_s.point.x < min_x):
                    min_x = pt_s.point.x

                if(pt_s.point.y < min_y):
                    min_y = pt_s.point.y

                if(pt_s.point.z < min_z):
                    min_z = pt_s.point.z

                if(pt_s.point.x > max_x):
                    max_x = pt_s.point.x

                if(pt_s.point.y > max_y):
                    max_y = pt_s.point.y

                if(pt_s.point.z > max_z):
                    max_z = pt_s.point.z

            print("RGB bbox: [" + str(rgb_min_x) + "," + str(rgb_min_y) + "," +str(rgb_max_x) + "," + str(rgb_max_y) + "]")
            print("3d bbox (map): [" + str(min_x) + "," + str(min_y) + "," +str(min_z) + "," + str(max_x) + "," + str(max_y) + ","+str(max_z)+"]")
            print("3d bbox (local): [" + str(local_min_x) + "," + str(local_min_y) + "," +str(local_min_z) + "," + str(local_max_x) + "," + str(local_max_y) + ","+str(local_max_z)+"]")

            x /= len(cur_cluster.data)
            y /= len(cur_cluster.data)
            z /= len(cur_cluster.data)

            x_local /= len(cur_cluster.data)
            y_local /= len(cur_cluster.data)
            z_local /= len(cur_cluster.data)

            print("3d centroid (local): [" + str(x_local) + "," + str(y_local) + "," + str(z_local))


            # centroid of the cluster
            ps_t = PointStamped()
            ps_t.header = "/map"
            ps_t.point.x = x
            ps_t.point.y = y
            ps_t.point.z = z

            cur_cluster.map_centroid = np.array((ps_t.point.x ,ps_t.point.y, ps_t.point.z))
            cur_cluster.local_centroid = np.array((x_local,y_local,z_local))

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            cur_cluster.raw_segmented_pc = pc2.create_cloud(header, cloud.fields, raw)

            #print("CLOUD HEADER:")
            #print(cloud.header)

            print("centroid:")
            # TODO: MAP CENTROID DOESN'T WORK ANY MORE
            print("map centroid:" + str(cur_cluster.map_centroid))
            print("local centroid:" + str(cur_cluster.local_centroid))

            cur_cluster.img_centroid = self.calculate_2d_image_centroid(cur_cluster.local_centroid)
            cur_cluster.img_bbox = [[rgb_min_x,rgb_min_y],[rgb_max_x,rgb_max_y]]

            bbmin = cur_cluster.img_bbox[0]
            bbmax = cur_cluster.img_bbox[1]

            print("bbmin: " + str(bbmin))
            print("bbmax: " + str(bbmax))

            bbox_width = abs(bbmax[0]-bbmin[0])
            bbox_height = abs(bbmax[1]-bbmin[1])

            print("bbw: " + str(bbox_width))
            print("bbh: " + str(bbox_height))


            # this next bit isn't very nice, but it crops out the cluster from the image
            # but, since converting clusters to 2d pixel co-ordinates can be a bit wonky sometimes
            # it entures that images will always be at least 64x64, or bigger if the clusters
            # are larger than this in screen co-ordinates.
            bx = cur_cluster.img_centroid[0]
            by = cur_cluster.img_centroid[1]

            #b_w = 64
            #b_h = b_w
            #if(bbox_width > b_w):
            #    b_w = bbox_width

            #if(bbox_width > b_h):
            #    b_h = bbox_width

            padding = 24

            y_start = rgb_min_y-padding
            y_end = rgb_max_y+padding

            x_start = rgb_min_x-padding
            x_end = rgb_max_x+padding

            if(y_end > 480):
                y_end = 480

            if(x_end > 640):
                x_end = 640

            if(y_start < 0):
                y_start = 0

            if(x_start < 0):
                x_start = 0

            cur_cluster.cv_image_cropped = cv_image[y_start:y_end, x_start:x_end]

            cur_cluster.cropped_image = bridge.cv2_to_imgmsg(cur_cluster.cv_image_cropped, encoding="bgr8")

            #success = cv2.imwrite(cid+'.jpeg',cv_image_cropped)

            #print("cropping succeded:" + str(success))


            print("img centroid: " + str(cur_cluster.img_centroid))
            print("img bbox: " + str(cur_cluster.img_bbox))

            bbox = BBox(min_x,max_x,min_y,max_y,min_z,max_z)

            x_range = max_x-min_x
            y_range = max_y-min_y
            z_range = max_z-min_z
            scale = 0.3

            x_mod = x_range*scale
            y_mod = y_range*scale
            z_mod = z_range*scale

            outer_core_bbox = BBox(min_x+x_mod,max_x-x_mod,min_y+y_mod,max_y-y_mod,min_z+z_mod,max_z-z_mod)

            cur_cluster.bbox = bbox
            cur_cluster.outer_core_bbox = outer_core_bbox

            if(talk): print("bbox: [" + str(bbox.x_min) + "," + str(bbox.y_min) + "," +str(bbox.z_min) + "," + str(bbox.x_max) + "," + str(bbox.y_max) + ","+str(bbox.z_max)+"]")

        #   if(talk): print("is centre point in bbox? " + str(cur_cluster.bbox.contains_point(cur_cluster.map_centroid)))
            self.cluster_list.append(cur_cluster)




class SOMAClusterTracker:

    def __init__(self):

        if(talk): print("--created cluster tracker--")
        self.segmentation_service = "/pcl_segmentation_service/pcl_segmentation"
        self.cur_scene = None
        self.prev_scene = None
        self.segmentation = SegmentationWrapper(self,self.segmentation_service)

    def add_unsegmented_scene(self,data):
        # takes in a SegmentedScene

        # segment the pc
        if(talk): print("waiting for segmentation service")
        print("segmenting (may take a second)")
        rospy.wait_for_service(self.segmentation_service)
        print("done")

        try:
            out = self.segmentation.seg_service(cloud=data)
        #    print("zzz")
        #    print("waiting for img")
        #    cur_img = rospy.wait_for_message("/pcl_segmentation_service/segmented_cloud_colored_img",Image)
        #    print("waiting for cld")
        #    cur_cld = rospy.wait_for_message("/pcl_segmentation_service/segmented_cloud_colored",sensor_msgs.msg.PointCloud2)
        #    print("done")

            #print("zzz")

            new_scene = SegmentedScene(out,data,self.segmentation.listener)

            print("new scene added, with " + str(new_scene.num_clusters) + " clusters")

            self.prev_scene = self.cur_scene
            self.cur_scene = new_scene

            if(self.prev_scene):
                print("we have a previous observation to compare to")
                tracker = VotingBasedClusterTrackingStrategy()
                tracker.track(self.cur_scene,self.prev_scene)
            else:
                print("no previous scene to compare to, skipping merging step, all clusters regarded as new")

        except rospy.ServiceException, e:
            if(talk): print("Failed Segmentation: ")
            if(talk): print(e)

    def add_segmented_scene(self,new_scene):
        # takes in a SegmentedScene
        if(talk): print("new scene added, with " + str(new_scene.num_clusters) + " clusters")
        self.prev_scene = self.cur_scene
        self.cur_scene = new_scene

        if(self.prev_scene):
            if(talk): print("we have a previous observation to compare to")
            tracker = NaiveClusterTrackingStrategy()
            tracker.track(self.cur_scene,self.prev_scene)
        #else:
        #    if(talk): print("no previous scene to compare to, skipping merging step")

class SegmentationWrapper:


    def __init__(self,ct,seg_serv):

        if(talk): print("loading wrapper")

        self.cluster_tracker = ct

        self.listener = tf.TransformListener()
        if(talk): print("waiting for tf")
        self.listener.waitForTransform("/head_xtion_rgb_optical_frame", "/map", rospy.Time(), rospy.Duration(4.0))
        if(talk): print("gotcha")

        #self.p_sub = rospy.Subscriber("/head_xtion/depth_registered/points", PointCloud2, self.cloud_callback)

        self.seg_service = rospy.ServiceProxy(seg_serv, segment)

    #    rospy.spin()


    def image_callback(self,data):
        #try:
            #bridge = CvBridge()
            #cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
            #cv.SaveImage("depth_camera_msg"+str(data.header.seq)+".png", cv.fromarray(cv_image))
            if(talk): print "mask saved!"
        #except CvBridgeError, e:
          #if(talk): print e

    def segment_callback(self,data):
    #    p = PointCloudTools(data)new_scene = SegmentedScene(out,data,self.segmentation.listener)
            if(talk): print("new scene added, with " + str(new_scene.num_clusters) + " clusters")

            self.prev_scene = self.cur_scene
            self.cur_scene = new_scene

            if(self.prev_scene):
                if(talk): print("we have a previous observation to compare to")
                tracker = NaiveClusterTrackingStrategy()
                tracker.track(self.cur_scene,self.prev_scene)
            else:
                if(talk): print("no previous scene to compare to, skipping merging step")


    def cloud_callback(self,data):
        #if(talk): print("got cloud:" + str(data.header.seq))
        try:
            out = self.seg_service(cloud=data)
            if(talk): print("segmentation complete")
            p = SegmentedScene(out,data,self.listener)
            self.cluster_tracker.add_segmented_scene(p)
            #if(talk): print(out)

        except rospy.ServiceException, e:
            if(talk): print "service call failed: %s"%e




if __name__ == '__main__':
    tracker = SOMAClusterTracker()
