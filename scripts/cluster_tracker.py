import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
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
        self.col_data = []
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


    def __init__(self,indices,cloud,listener,col_image,col_cloud):
        if(talk): print("\nthis cloud has " + str(len(indices.clusters_indices)) + " clusters")
        self.num_clusters = len(indices.clusters_indices)
        self.cloud = cloud

        self.col_seg_image = col_image
        self.col_seg_cloud = pc2.read_points(col_cloud)
        col_data = list(self.col_seg_cloud)

        colours = set()

        for x in col_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            #print r,g,b # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            colours.add((r,g,b))



            

        print("found cols: " + str(colours))
        #print("reading seg mask")
        #self.seg_mask_cloud = pc2.read_points(ccld)
        #print("done")

        #if(talk): print(cloud.header)
        self.raw_cloud = pc2.read_points(cloud)
        int_data = list(self.raw_cloud)
        #if(talk): print("data: " + str(len(int_data)))

        self.cluster_list = []


        if(talk): print("loading clusters")


        for root_cluster in indices.clusters_indices:
            if(talk): print("--- CLUSTER ----")

            #cid = [random.choice("ABCDEFGHIGJSDHS903R4IDFA34RAFDAFDLLAFD") for _ in range(4)] # this call makes me very nervous
            #cid = ''.join(cid)
            cid = str(uuid.uuid4()) # str so we can later link it to a soma2 object, which indexes by string
            if(talk): print("randomly assigned temporary cid: " + str(cid))
            cur_cluster = SegmentedCluster(cid,root_cluster)


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
            min_x = 99999
            min_y = 99999
            min_z = 99999

            max_x = -99999
            max_y = -99999
            max_z = -99999

            translation,rotation = listener.lookupTransform("/map", "/head_xtion_rgb_optical_frame", rospy.Time())
            trans_matrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

            #if(talk): print("got transform: ")
            #if(talk): print(trans_cache[0])

            for points in cur_cluster.data:

                # store the root points

                # and store the world transformed point too
                pt_s = PointStamped()
                pt_s.header = "/map"
                pt_s.point.x = points[0]
                pt_s.point.y = points[1]
                pt_s.point.z = points[2]

                x_local += pt_s.point.x
                y_local += pt_s.point.y
                z_local += pt_s.point.z

                xyz = tuple(np.dot(trans_matrix, np.array([pt_s.point.x, pt_s.point.y, pt_s.point.z, 1.0])))[:3]
                pt_s.point = geometry_msgs.msg.Point(*xyz)

                cur_cluster.data_world.append(pt_s)

                x += pt_s.point.x
                y += pt_s.point.y
                z += pt_s.point.z

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


            #if(talk): print("bbox: [" + str(min_x) + "," + str(min_y) + "," +str(min_z) + "," + str(max_x) + "," + str(max_y) + ","+str(max_z)+"]")

            x /= len(cur_cluster.data)
            y /= len(cur_cluster.data)
            z /= len(cur_cluster.data)

            x_local /= len(cur_cluster.data)
            y_local /= len(cur_cluster.data)
            z_local /= len(cur_cluster.data)

            # centroid of the cluster
            ps_t = PointStamped()
            ps_t.header = "/map"
            ps_t.point.x = x
            ps_t.point.y = y
            ps_t.point.z = z

            cur_cluster.map_centroid = np.array((ps_t.point.x ,ps_t.point.y, ps_t.point.z))
            cur_cluster.local_centroid = np.array((x_local,y_local,z_local))


            if(talk): print("centroid:")
            if(talk): print(cur_cluster.map_centroid)
            bbox = BBox(min_x,max_x,min_y,max_y,min_z,max_z)
            cur_cluster.bbox = bbox
            if(talk): print("bbox: [" + str(bbox.x_min) + "," + str(bbox.y_min) + "," +str(bbox.z_min) + "," + str(bbox.x_max) + "," + str(bbox.y_max) + ","+str(bbox.z_max)+"]")

            if(talk): print("is centre point in bbox? " + str(cur_cluster.bbox.contains_point(cur_cluster.map_centroid)))
            self.cluster_list.append(cur_cluster)




class SOMAClusterTracker:

    def __init__(self):

        if(talk): print("--created cluster tracker--")
        self.segmentation_service = "/pcl_segmentation_service/pcl_segmentation"
        self.cur_scene = None
        self.prev_scene = None
        self.segmentation = SegmentationWrapper(self,self.segmentation_service)

        self.col_cloud_sub = rospy.Subscriber("/pcl_segmentation_service/segmented_cloud_colored", PointCloud2, self.cloud_cb)
        self.col_img_sub =  rospy.Subscriber("/pcl_segmentation_service/segmented_cloud_colored_img", Image, self.image_cb)

    def image_cb(self, data):
        self.cur_seg_color_image = data

    def cloud_cb(self, data):
        self.cur_seg_color_cloud = data

    def add_unsegmented_scene(self,data):
        # takes in a SegmentedScene

        # segment the pc
        if(talk): print("waiting for segmentation service")
        rospy.wait_for_service(self.segmentation_service)

        try:
            out = self.segmentation.seg_service(cloud=data)
            #print("zzz")

            new_scene = SegmentedScene(out,data,self.segmentation.listener,self.cur_seg_color_image,self.cur_seg_color_cloud)

            if(talk): print("new scene added, with " + str(new_scene.num_clusters) + " clusters")

            self.prev_scene = self.cur_scene
            self.cur_scene = new_scene

            if(self.prev_scene):
                if(talk): print("we have a previous observation to compare to")
                tracker = NaiveClusterTrackingStrategy()
                tracker.track(self.cur_scene,self.prev_scene)
            else:
                if(talk): print("no previous scene to compare to, skipping merging step")

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
