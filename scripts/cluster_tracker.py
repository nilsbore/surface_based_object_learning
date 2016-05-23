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
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
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
    def __init__(self,idc,cluster_indices):
        self.data = []
        self.cluster_indices = cluster_indices
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


    #def calculate_2d_image_centroid(self,centroid):
    #    model = image_geometry.PinholeCameraModel()
    #    rospy.loginfo("waiting for camera")
    #    self.camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=3.0)
    #    model.fromCameraInfo(self.camera_msg)
    #    px = model.project3dToPixel((centroid[0], centroid[1], centroid[2]))
    #    return px

    def calculate_2d_image_centroid(self,bbox_min,bbox_max):
        min_x = bbox_min[0]
        max_x = bbox_max[0]
        min_y = bbox_min[1]
        max_y = bbox_max[1]

        width = abs(max_x-min_x)
        height = abs(max_y-min_y)

        return [width/2,height/2]


    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))

    def transform_frame_to_map(self,cloud):
        rospy.loginfo("to map")

        t = self.listener.getLatestCommonTime("map", self.child_camera_frame)
        self.listener.waitForTransform("map", self.child_camera_frame, t, rospy.Duration(5.0))
        tr_r = self.listener.lookupTransform("map", self.child_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = 'map'
        tr_s.child_frame_id = self.child_camera_frame
        tr_s.transform = tr

        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res

    def transform_cloud_to_base(self,cloud):
        rospy.loginfo("to base")

        t = self.listener.getLatestCommonTime("base_link", self.child_camera_frame)
        self.listener.waitForTransform("base_link", self.child_camera_frame, t, rospy.Duration(15.0))
        tr_r = self.listener.lookupTransform("base_link", self.child_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = 'base_link'
        tr_s.child_frame_id = self.child_camera_frame
        tr_s.transform = tr

        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res

    def transform_cloud_to_frame(self,cloud):
        rospy.loginfo("to frame")

        t = self.listener.getLatestCommonTime(self.child_camera_frame, self.root_camera_frame)
        self.listener.waitForTransform(self.child_camera_frame, self.root_camera_frame, t, rospy.Duration(5.0))
        tr_r = self.listener.lookupTransform(self.child_camera_frame, self.root_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = self.child_camera_frame
        tr_s.child_frame_id = self.root_camera_frame
        tr_s.transform = tr

        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res

    def get_camera_info_topic(self):
        self.camera_msg = None
        try:
            self.camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=2)
        except Exception,e:
            rospy.loginfo("couldn't find /head_xtion/depth_registered/camera_info")

        if(self.camera_msg):
            rospy.loginfo("found topic: /head_xtion/depth_registered/camera_info")
            return self.camera_msg

        try:
            self.camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/sw_registered/camera_info",  CameraInfo, timeout=2)
        except Exception,e:
            rospy.loginfo("couldn't find /head_xtion/depth_registered/sw_registered/camera_info")

        if(self.camera_msg):
            rospy.loginfo("found topic: /head_xtion/depth_registered/sw_registered/camera_info")
            return self.camera_msg

        return None

    def get_moore_neighbourhood(self,point):
        y = point[0]
        x = point[1]
        points = []
        points.append(point)

        if not(y - 1 < 0):
            points.append([y-1,x])

        if not(y + 1 > 480):
            points.append([y+1,x])

        if not(x - 1 < 0):
            points.append([y,x-1])

        if not(y + 1 > 640):
            points.append([y,x+1])

        return points

    def set_frames(self,cloud):
        rospy.loginfo("RUNNING SET FRAMES")
        self.root_camera_frame = ""
        self.child_camera_frame = ""
        rospy.loginfo("camera input:" + str(cloud.header.frame_id))

        if("head_xtion_rgb_optical_frame" in str(cloud.header.frame_id)):
            rospy.loginfo("found head_xtion_rgb_optical_frame")
            self.root_camera_frame = cloud.header.frame_id
            self.child_camera_frame = "head_xtion_rgb_frame"

        if("head_xtion_depth_optical_frame" in str(cloud.header.frame_id)):
            rospy.loginfo("found head_xtion_depth_optical_frame")
            self.root_camera_frame = cloud.header.frame_id
            self.child_camera_frame = "head_xtion_depth_frame"

        rospy.loginfo("frames are:")
        rospy.loginfo(self.root_camera_frame)
        rospy.loginfo(self.child_camera_frame)


    def __init__(self,indices,input_scene_cloud,pub):
        self.set_frames(input_scene_cloud)
        self.clean_setup = False
        #rospy.loginfo("\nthis cloud has " + str(len(indices.clusters_indices)) + " clusters")
        self.num_clusters = len(indices.clusters_indices)
        self.input_scene_cloud = input_scene_cloud
        self.listener = tf.TransformListener()

        # let the listener grab a few frames of tf
        rospy.sleep(1)

        self.camera_msg = self.get_camera_info_topic()
        if(self.camera_msg is None):
            rospy.logerr("Unable to locate camera_info topic")
            rospy.logerr("Exiting")
            return

        #rospy.loginfo("waiting for transform")
        try:
            t = self.listener.getLatestCommonTime("map", self.root_camera_frame)
            self.listener.waitForTransform("map", self.root_camera_frame, t, rospy.Duration(10.0))
        except rospy.ServiceException, e:
            rospy.logerr("Unable to find transform between camera frame and map frame")
            rospy.loginfo(e)
            return

        #rospy.loginfo("---- INPUT HEADER: ----")
        #rospy.loginfo(input_scene_cloud.header)
        #rospy.loginfo("POINTS IN INPUT: " + str(len(input_scene_cloud.data)))
        self.waypoint = "None"
        t = self.listener.getLatestCommonTime("map", self.root_camera_frame)
        translation,rotation = self.listener.lookupTransform("map", self.root_camera_frame, t)

        self.raw_cloud = pc2.read_points(input_scene_cloud)
        int_data = list(self.raw_cloud)

        self.cluster_list = []

        #rospy.loginfo("getting image of scene")
        scene_img = rospy.wait_for_message("/head_xtion/rgb/image_rect_color",  Image, timeout=15.0)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(scene_img, desired_encoding="bgr8")
        rospy.loginfo("got it")

        to_frame = self.transform_cloud_to_frame(input_scene_cloud)
        to_map = self.transform_frame_to_map(to_frame)
        map_points = pc2.read_points(to_map)
        map_points_int_data = list(map_points)

        # rospy.loginfo("loading clusters")
        for root_cluster in indices.clusters_indices:
            map_points_data = []
            rgb_mask = np.zeros(cv_image.shape,np.uint8)
            rospy.loginfo("--- CLUSTER ----")

            cid = str(uuid.uuid4()) # str so we can later link it to a soma2 object, which indexes by string
            #rospy.loginfo("randomly assigned temporary cid: " + str(cid))
            cur_cluster = SegmentedCluster(cid,root_cluster)
            #raw = []
            for idx in root_cluster.data:
                cur_cluster.data.append(int_data[idx])
                map_points_data.append(map_points_int_data[idx])

            #rospy.loginfo("i added: " + str(len(cur_cluster.data)) + " points to a cluster")

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

            trans_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))
            #rospy.loginfo("trans_matrix: ")
            #rospy.loginfo(trans_matrix)

            # rospy.loginfo("got transform: ")
            # rospy.loginfo(trans_cache[0])

            cluster_camframe = []

            model = image_geometry.PinholeCameraModel()
            #rospy.loginfo("waiting for camera_info topic")


            model.fromCameraInfo(self.camera_msg)

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
                color_data = points[3]

                cluster_camframe.append((pt_s.point.x,pt_s.point.y,pt_s.point.z,color_data))

                # get the rgb pos of the point
                rgb_point = model.project3dToPixel((pt_s.point.x, pt_s.point.y, pt_s.point.z))
                #rospy.loginfo("rgb point: " + str(rgb_point))

                rgb_x = rgb_point[0]
                rgb_y = rgb_point[1]

                # add this to the mask
                # this is technically a bit hacky and slow, but on the scale we're doing it
                # it really doesn't matter
                for p in self.get_moore_neighbourhood([rgb_y,rgb_x]):
                    rgb_mask[p[0],p[1]] = [255,255,255]

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

            #rospy.loginfo("RGB bbox: [" + str(rgb_min_x) + "," + str(rgb_min_y) + "," +str(rgb_max_x) + "," + str(rgb_max_y) + "]")
            #rospy.loginfo("3d bbox (map): [" + str(min_x) + "," + str(min_y) + "," +str(min_z) + "," + str(max_x) + "," + str(max_y) + ","+str(max_z)+"]")
            #rospy.loginfo("3d bbox (local): [" + str(local_min_x) + "," + str(local_min_y) + "," +str(local_min_z) + "," + str(local_max_x) + "," + str(local_max_y) + ","+str(local_max_z)+"]")

            x /= len(cur_cluster.data)
            y /= len(cur_cluster.data)
            z /= len(cur_cluster.data)

            x_local /= len(cur_cluster.data)
            y_local /= len(cur_cluster.data)
            z_local /= len(cur_cluster.data)

            #rospy.loginfo("3d centroid (local): [" + str(x_local) + "," + str(y_local) + "," + str(z_local))


            # centroid of the cluster
            ps_t = PointStamped()
            ps_t.header = "/map"
            ps_t.point.x = x
            ps_t.point.y = y
            ps_t.point.z = z

            cur_cluster.map_centroid = np.array((ps_t.point.x ,ps_t.point.y, ps_t.point.z))
            cur_cluster.local_centroid = np.array((x_local,y_local,z_local))


            header_cam = std_msgs.msg.Header()
            header_cam.stamp = rospy.Time.now()
            header_cam.frame_id = self.root_camera_frame
            cur_cluster.segmented_pc_camframe = pc2.create_cloud(header_cam, input_scene_cloud.fields, cluster_camframe)



            header_map = std_msgs.msg.Header()
            header_map.stamp = rospy.Time.now()
            header_map.frame_id = 'map'
            cur_cluster.segmented_pc_mapframe = pc2.create_cloud(header_map, to_map.fields, map_points_data)
            pub.publish(to_map)


            #rospy.loginfo("centroid:")
            # TODO: MAP CENTROID DOESN'T WORK ANY MORE
            #rospy.loginfo("map centroid:" + str(cur_cluster.map_centroid))
            #rospy.loginfo("local centroid:" + str(cur_cluster.local_centroid))


            cur_cluster.img_bbox = [[rgb_min_x,rgb_min_y],[rgb_max_x,rgb_max_y]]
            cur_cluster.img_centroid = self.calculate_2d_image_centroid(cur_cluster.img_bbox[0],cur_cluster.img_bbox[1])


            bbmin = cur_cluster.img_bbox[0]
            bbmax = cur_cluster.img_bbox[1]

            #rospy.loginfo("bbmin: " + str(bbmin))
            #rospy.loginfo("bbmax: " + str(bbmax))

            bbox_width = abs(bbmax[0]-bbmin[0])
            bbox_height = abs(bbmax[1]-bbmin[1])

            #rospy.loginfo("bbw: " + str(bbox_width))
            #rospy.loginfo("bbh: " + str(bbox_height))


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

            cur_cluster.rgb_mask = bridge.cv2_to_imgmsg(rgb_mask, encoding="bgr8")
            #success = cv2.imwrite(cid+'.jpeg',rgb_mask)

            #rospy.loginfo("cropping succeded:" + str(success))


            #rospy.loginfo("img centroid: " + str(cur_cluster.img_centroid))
            #rospy.loginfo("img bbox: " + str(cur_cluster.img_bbox))

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

            # rospy.loginfo("bbox: [" + str(bbox.x_min) + "," + str(bbox.y_min) + "," +str(bbox.z_min) + "," + str(bbox.x_max) + "," + str(bbox.y_max) + ","+str(bbox.z_max)+"]")

        #    rospy.loginfo("is centre point in bbox? " + str(cur_cluster.bbox.contains_point(cur_cluster.map_centroid)))
            self.cluster_list.append(cur_cluster)
            self.clean_setup = True



class SOMAClusterTracker:

    def __init__(self):

        rospy.loginfo("--created cluster tracker--")
        self.segmentation_service = "/pcl_segmentation_service/pcl_segmentation"
        self.cur_scene = None
        self.prev_scene = None
        self.segmentation = SegmentationWrapper(self,self.segmentation_service)
        self.pub = rospy.Publisher('/world_modeling/cluster_tracker_intermediate', PointCloud2, queue_size=10)

    def reset(self):
        self.cur_scene = None
        self.prev_scene = None

    def add_unsegmented_scene(self,data):
        # takes in a SegmentedScene
        rospy.loginfo("\n\n--- Beginning Interpretation of Scene --")
        # segment the pc
        rospy.loginfo("waiting for segmentation service")
        rospy.loginfo("segmenting (may take a second)")
        rospy.wait_for_service(self.segmentation_service)
        rospy.loginfo("segmentation done")

        try:
            out = self.segmentation.seg_service(cloud=data)

            new_scene = SegmentedScene(out,data,self.pub)

            rospy.loginfo("new scene added, with " + str(new_scene.num_clusters) + " clusters")
            self.cur_scene = new_scene

            if(self.prev_scene):
                rospy.loginfo("we have a previous observation to compare to")
                tracker = VotingBasedClusterTrackingStrategy()
                tracker.track(self.cur_scene,self.prev_scene)
            else:
                rospy.loginfo("no previous scene to compare to, skipping merging step, all clusters regarded as new")
        except rospy.ServiceException, e:
             rospy.logerr("Failed Segmentation: ")
             rospy.logerr(e)

        self.prev_scene = self.cur_scene
        return new_scene

    def add_segmented_scene(self,new_scene):
        # takes in a SegmentedScene
        rospy.loginfo("new scene added, with " + str(new_scene.num_clusters) + " clusters")
        self.prev_scene = self.cur_scene
        self.cur_scene = new_scene

        if(self.prev_scene):
            rospy.loginfo("we have a previous observation to compare to")
            tracker = NaiveClusterTrackingStrategy()
            tracker.track(self.cur_scene,self.prev_scene)
        #else:
        #     rospy.loginfo("no previous scene to compare to, skipping merging step")

class SegmentationWrapper:


    def __init__(self,ct,seg_serv):
        rospy.loginfo("loading wrapper")
        self.cluster_tracker = ct
        self.seg_service = rospy.ServiceProxy(seg_serv, segment)

    def image_callback(self,data):
        #try:
            #bridge = CvBridge()
            #cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
            #cv.SaveImage("depth_self.camera_msg"+str(data.header.seq)+".png", cv.fromarray(cv_image))
             print "mask saved!"
        #except CvBridgeError, e:
          # print e

    def segment_callback(self,data):
    #    p = PointCloudTools(data)new_scene = SegmentedScene(out,data,self.segmentation.listener)
            rospy.loginfo("new scene added, with " + str(new_scene.num_clusters) + " clusters")

            self.prev_scene = self.cur_scene
            self.cur_scene = new_scene

            if(self.prev_scene):
                rospy.loginfo("we have a previous observation to compare to")
                tracker = NaiveClusterTrackingStrategy()
                tracker.track(self.cur_scene,self.prev_scene)
            else:
                 rospy.loginfo("no previous scene to compare to, skipping merging step")


    def cloud_callback(self,data):
        # rospy.loginfo("got cloud:" + str(data.header.seq))
        try:
            out = self.seg_service(cloud=data)
            rospy.loginfo("segmentation complete")
            p = SegmentedScene(out,data)
            self.cluster_tracker.add_segmented_scene(p)
            # rospy.loginfo(out)

        except rospy.ServiceException, e:
             print "service call failed: %s"%e




if __name__ == '__main__':
    tracker = SOMAClusterTracker()
