#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import String
import cv
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from segmentation_srv_definitions.srv import *
import numpy as np
import geometry_msgs
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
import tf2_ros
import tf, tf2_msgs.msg
import PyKDL
from cluster_tracking_strategies import *
import uuid
import random
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import base64
from roi_filter import ROIFilter
from view_registration import ViewAlignmentManager
#import python_pcd
#import pcl
#from bham_seg import Segmentation
from util import TransformationStore
from bham_seg_filter.srv import *
from surface_based_object_learning.srv import *

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


class SegmentedCluster:
    def __init__(self,segment_indices):
        self.data = []
        self.segment_indices = segment_indices
        self.data_world = []
        self.map_centroid = None
        self.local_centroid = None
        self.bbox = None
        self.assigned = False
        self.label = None
        self.confidence = 0.0
        self.segment_id = "INSTANCE-ID-"+str(uuid.uuid4())


    def reset_assignment(self):
        self.assigned = False

class SegmentedScene:

    def reset_segment_assignments(self):
        for segment in self.segment_list:
            segment.reset_assignment()

    def contains_segment_id(self,id):
        for segment in self.segment_list:
            if(segment.segment_id == id):
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

    def transform_cloud(self,cloud):
        rospy.loginfo("to map")

        t = self.transformation_store.getLatestCommonTime(self.target_camera_frame, self.starting_camera_frame)
        #self.transformation_store.waitForTransform(self.target_camera_frame, self.starting_camera_frame, t, rospy.Duration(5.0))
        tr_r = self.transformation_store.lookupTransform(self.target_camera_frame, self.starting_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
        self.transform_frame_to_map = tr

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = self.target_camera_frame
        tr_s.child_frame_id = self.starting_camera_frame
        tr_s.transform = tr


        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(cloud.header, cloud.fields, points_out)
        rospy.loginfo("done")
        return res

    def set_frames(self,cloud):
        rospy.loginfo("RUNNING SET FRAMES")
        self.starting_camera_frame = cloud.header.frame_id
        self.target_camera_frame = "map"

        rospy.loginfo("frames are:")
        rospy.loginfo(self.starting_camera_frame)
        rospy.loginfo(self.target_camera_frame)


    def __init__(self,indices,working_cloud,observation_data,roi_filter,offline_data=None):
        self.set_frames(working_cloud)
        self.unfiltered_cloud = observation_data['scene_cloud']
        self.scene_id = str(uuid.uuid4())
        self.clean_setup = False
        self.num_segments = len(indices)

        self.transformation_store = tf.TransformerROS()
        for transform in observation_data['tf'].transforms:
            self.transformation_store.setTransform(transform)

        self.label = "unknown"
        self.confidence = 0.0
        self.segment_list = []
        self.offline_observation_data = offline_data

        rospy.loginfo("Getting camera msg")
        if(self.offline_observation_data is None):
            self.camera_msg = observation_data['camera_info']
            if(self.camera_msg is None):
                rospy.logerr("Unable to locate camera_info topic. This is fatal.")
                rospy.logerr("Exiting")
                return
        else:
            rospy.loginfo("Using pre-set data")

        self.waypoint = observation_data['waypoint']

        self.raw_cloud = pc2.read_points(working_cloud)
        int_data = list(self.raw_cloud)


        if(self.offline_observation_data is None):
            scene_rgb_img = observation_data['rgb_image']
            scene_depth_img = observation_data['depth_image']
            translation,rotation = self.transformation_store.lookupTransform("map", self.starting_camera_frame, rospy.Time())
        else:
            scene_rgb_img = self.offline_observation_data['rgb_image']
            scene_depth_img = self.offline_observation_data['depth_image']
            tf_st = TransformationStore().msg_to_transformer(self.offline_observation_data['tf'])
            self.transformation_store = tf_st
            translation,rotation = tf_st.lookupTransform("map", self.starting_camera_frame, rospy.Time())
            self.camera_msg = self.offline_observation_data['camera_info']


        self.to_map_trans = translation
        self.to_map_rot = rotation

        bridge = CvBridge()
        cv_rgb_image = bridge.imgmsg_to_cv2(scene_rgb_img)
        cv_depth_image = bridge.imgmsg_to_cv2(scene_depth_img)

        #print("writing cur scene")
        #f = cv2.imwrite('stuff.png',cv_rgb_image)
        #print(f)


        to_map = self.transform_cloud(working_cloud)

        map_points = pc2.read_points(to_map)
        map_points_int_data = list(map_points)

        # rospy.loginfo("loading segments")
        rospy.loginfo("Located: %d candidate segments", len(indices))
        roi_filter.gather_rois()
        for root_segment in indices:
            map_points_data = []
            image_mask = np.zeros(cv_rgb_image.shape,np.uint8)

            rospy.loginfo("CLUSTER SIZE: " + str(len(root_segment.data)))
            if(len(root_segment.data) > 500 and len(root_segment.data) < 5000):
                rospy.loginfo("cluster looks like the right size")
            else:
                rospy.loginfo("cluster not the right size")
                continue

            rospy.loginfo("--- segment ----")

            cur_segment = SegmentedCluster(root_segment.data)

            for idx in root_segment.data:
                cur_segment.data.append(int_data[idx])
                map_points_data.append(map_points_int_data[idx])

            x = 0
            y = 0
            z = 0

            x_local = 0
            y_local = 0
            z_local = 0

            min_x=min_y=min_z=99999
            max_x=max_y=max_z = -99999
            local_min_x=local_min_y=local_min_z = 99999
            local_max_x=local_max_y=local_max_z = -99999

            trans_matrix = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation))

            segment_camframe = []

            model = image_geometry.PinholeCameraModel()

            model.fromCameraInfo(self.camera_msg)

            rgb_min_x = 90000
            rgb_max_x = 0
            rgb_min_y = 90000
            rgb_max_y = 0
            points_in_roi = 0
            total_points = 0
            sc_roi_check = False
            unique_rgb = set()
            rgb_points = []


            for points in cur_segment.data:




                # store the roxe world transformed point too
                pt_s = PointStamped()
                pt_s.header = "/map"
                pt_s.point.x = points[0]
                pt_s.point.y = points[1]
                pt_s.point.z = points[2]
                color_data = points[3]
                total_points+=1

                segment_camframe.append((pt_s.point.x,pt_s.point.y,pt_s.point.z,color_data))

                # get the 2d pos of the 3d point
                rgb_point = model.project3dToPixel((pt_s.point.x, pt_s.point.y, pt_s.point.z))

                rgb_x = int(rgb_point[0])
                rgb_y = int(rgb_point[1])
                rgb_points.append(rgb_point)
                image_mask[rgb_y,rgb_x] = [255,255,255]

                unique_rgb.add((rgb_x,rgb_y))
                # next we extend a little bit around the point and add some neighbouring points
                # in to make the mask fill the object better


                #for p in self.get_moore_neighbourhood([rgb_y,rgb_x]):
                #    image_mask[int(p[0]),int(p[1])] = [255,255,255]


                # figure out a bounding box for this image
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

                # here we check to make sure the object has at least one point in a SOMa ROI
                if(sc_roi_check is False):
                    pir = roi_filter.accel_point_in_roi([pt_s.point.x,pt_s.point.y])
                    if(pir):
                        sc_roi_check = True
                        points_in_roi+=1

                cur_segment.data_world.append(pt_s)

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

            if(points_in_roi == 0):
                rospy.loginfo("Not enough of object in ROI to continue")
                continue
            else:
                rospy.loginfo("Object is in ROI")

            x /= len(cur_segment.data)
            y /= len(cur_segment.data)
            z /= len(cur_segment.data)

            x_local /= len(cur_segment.data)
            y_local /= len(cur_segment.data)
            z_local /= len(cur_segment.data)

            #rospy.loginfo("3d centroid (local): [" + str(x_local) + "," + str(y_local) + "," + str(z_local))

            # centroid of the segment
            ps_t = PointStamped()
            ps_t.header = "/map"
            ps_t.point.x = x
            ps_t.point.y = y
            ps_t.point.z = z

            ws_pose = geometry_msgs.msg.Pose()
            ws_pose.position.x = ps_t.point.x
            ws_pose.position.y = ps_t.point.y
            ws_pose.position.z = ps_t.point.z

            cur_segment.map_centroid = ws_pose

            cur_segment.local_centroid = np.array((x_local,y_local,z_local))


            header_cam = std_msgs.msg.Header()
            header_cam.stamp = rospy.Time.now()
            header_cam.frame_id = self.starting_camera_frame
            cur_segment.segmented_pc_camframe = pc2.create_cloud(header_cam, working_cloud.fields, segment_camframe)

            header_map = std_msgs.msg.Header()
            header_map.stamp = rospy.Time.now()
            header_map.frame_id = 'map'
            cur_segment.segmented_pc_mapframe = pc2.create_cloud(header_map, to_map.fields, map_points_data)
            rospy.loginfo("segment has: " + str(len(map_points_data)) + " points")

            rospy.loginfo("map centroid:" + str(cur_segment.map_centroid))

            cur_segment.img_bbox = [[rgb_min_x,rgb_min_y],[rgb_max_x,rgb_max_y]]
            cur_segment.img_centroid = self.calculate_2d_image_centroid(cur_segment.img_bbox[0],cur_segment.img_bbox[1])

            bbmin = cur_segment.img_bbox[0]
            bbmax = cur_segment.img_bbox[1]

            #rospy.loginfo("bbmin: " + str(bbmin))
            #rospy.loginfo("bbmax: " + str(bbmax))

            bbox_width = abs(bbmax[0]-bbmin[0])
            bbox_height = abs(bbmax[1]-bbmin[1])

            #rospy.loginfo("bbw: " + str(bbox_width))
            #rospy.loginfo("bbh: " + str(bbox_height))


            # this next bit isn't very nice, but it crops out the segment from the image
            # but, since converting segments to 2d pixel co-ordinates can be a bit wonky sometimes
            # it entures that images will always be at least 64x64, or bigger if the segments
            # are larger than this in screen co-ordinates.
            bx = cur_segment.img_centroid[0]
            by = cur_segment.img_centroid[1]

            #b_w = 64
            #b_h = b_w
            #if(bbox_width > b_w):
            #    b_w = bbox_width

            #if(bbox_width > b_h):
            #    b_h = bbox_width

            padding = 32

            cur_segment.cv_rgb_image_cropped_unpadded = cv_rgb_image[int(rgb_min_y):int(rgb_max_y), int(rgb_min_x):int(rgb_max_x)]

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

            do_luminance_filtering = False


            cur_segment.cv_rgb_image_cropped = cv_rgb_image[int(y_start):int(y_end), int(x_start):int(x_end)]
            cur_segment.cv_depth_image_cropped = cv_depth_image[int(y_start):int(y_end), int(x_start):int(x_end)]

            cur_segment.cropped_depth_image = bridge.cv2_to_imgmsg(cur_segment.cv_depth_image_cropped)

            cur_segment.cropped_rgb_image = bridge.cv2_to_imgmsg(cur_segment.cv_rgb_image_cropped)


            image_mask = cv2.cvtColor(image_mask,cv2.COLOR_BGR2GRAY)
            ret,thresh = cv2.threshold(image_mask,127,255,cv2.THRESH_BINARY)
            contours,hierarchy=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            image_mask = np.zeros(cv_rgb_image.shape,np.uint8)
            cv2.drawContours(image_mask, contours, -1, (255,255,255), 3)

            cur_segment.image_mask = bridge.cv2_to_imgmsg(image_mask)

            cv2.imwrite(str(uuid.uuid4())+'_mask.png',image_mask)



            if(do_luminance_filtering):
                hsv = cv2.cvtColor(cur_segment.cv_rgb_image_cropped_unpadded, cv2.COLOR_BGR2YUV)

                avg_luma = 0
                for pixel in hsv:
                    luma = pixel[0]
                    avg_luma += luma[0]
                    avg_luma += luma[1]
                    avg_luma += luma[2]

                avg_luma /= len(hsv)
                print("AVG LUMA: " + str(avg_luma))

                al = 0
                h,w,bpp = np.shape(cur_segment.cv_rgb_image_cropped_unpadded)
                pc = 0
                for py in range(0,h):
                    for px in range(0,w):
                        pixel = cur_segment.cv_rgb_image_cropped_unpadded[py][px]
                        r = pixel[2]
                        g = pixel[1]
                        b = pixel[0]
                        lm = 0.299*r + 0.587*g + 0.144*b
                        al+=lm
                        pc+=1

                un_px = len(unique_rgb)
                print("PIXELS: " + str(pc))
                print("UNIQUE PIXELS: " + str(un_px))
                print("SUM LUMA: " + str(al))
                al /= pc
                print("AVG LUMA: " + str(al))

                #f = cv2.imwrite("obj_segments_image/pixels/"+str(int(un_px))+'.q',cur_segment.cv_rgb_image_cropped_unpadded)
                #print("done writing")
                #print(f)
                #cv2.imwrite("obj_segments_image/luma/"+str(int(al))+'.jpeg',cur_segment.cv_rgb_image_cropped_unpadded)

                if(al > 50 and un_px > 1800 and un_px < 15000):
                    print("object meets our luminance criteria!")
                else:
                    print("ignoring this object, looks like garbage!")
                    continue

            #f = cv2.imwrite('obj_segments_image/aaa_CROPPED.jpeg',cur_segment.cv_rgb_image_cropped_unpadded)
            #print(f)

            #python_pcd.write_pcd("obj_segments_cloud/"+cid+".pcd",cur_segment.segmented_pc_camframe)

            #rospy.loginfo("cropping succeded:" + str(success))

            #rospy.loginfo("img centroid: " + str(cur_segment.img_centroid))
            #rospy.loginfo("img bbox: " + str(cur_segment.img_bbox))

            bbox = BBox(min_x,max_x,min_y,max_y,min_z,max_z)
            bbox_local = BBox(local_min_x,local_max_x,local_min_y,local_max_y,local_min_z,local_max_z)

            x_range = max_x-min_x
            y_range = max_y-min_y
            z_range = max_z-min_z
            scale = 0.3

            x_mod = x_range*scale
            y_mod = y_range*scale
            z_mod = z_range*scale

            outer_core_bbox = BBox(min_x+x_mod,max_x-x_mod,min_y+y_mod,max_y-y_mod,min_z+z_mod,max_z-z_mod)

            cur_segment.bbox = bbox
            cur_segment.bbox_local = bbox_local
            cur_segment.outer_core_bbox = outer_core_bbox


            self.segment_list.append(cur_segment)

        self.clean_setup = True


class SegmentProcessor:

    def __init__(self):
        #rospy.init_node('CT_TEST_NODE', anonymous = True)
        rospy.loginfo("--created segment tracker--")
        self.seg_srv_topic = "/bham_filtered_segmentation/segment"
        self.cur_scene = None
        self.prev_scene = None
        self.root_scene = None
        self.roi_filter = ROIFilter()
        self.view_alignment_manager = ViewAlignmentManager()
        self.segmentation_service = rospy.ServiceProxy(self.seg_srv_topic, bham_seg, 10)

    def reset(self):
        self.cur_scene = None
        self.prev_scene = None
        self.root_scene = None

    def segment_scene(self,input_cloud,robot_pos=None):

        if(robot_pos is None):
            rospy.wait_for_service('/get_closest_roi_to_robot',10)
            roicl = rospy.ServiceProxy('/get_closest_roi_to_robot',GetROIClosestToRobot)
            rp = rospy.wait_for_message("/robot_pose",Pose,10)
            roip = roicl(pose=rp.position)
            robot_pos = roip.output


        output = self.segmentation_service(cloud=input_cloud,posearray=robot_pos)
        return output


    def add_unsegmented_scene(self,observation_data,offline_data=None):
        rospy.loginfo("\n\n--- Beginning Interpretation of Scene --")
        rospy.loginfo("filtering this cloud by SOMA ROI")
        rospy.loginfo("segmenting (may take a second)")

        robot_pos = None
        if(offline_data is not None):
            robot_pos = offline_data['robot_pose']
        segment_response = self.segment_scene(observation_data['scene_cloud'],robot_pos)
        if(segment_response.clusters_indices is None):
            rospy.logerr("No indices in segmented point cloud. Quitting processing this view.")

        # this might be different to what is in the raw observation message
        # as we may do things like cut it off after a certain distance etc.
        # during segmentation
        working_cloud = observation_data['scene_cloud']

        # this is pretty standard, we need the indices of the clusters
        indices = segment_response.clusters_indices
        rospy.loginfo("found:" + str(len(indices)) + " clusters")
        rospy.loginfo("segmentation done")
        new_scene = SegmentedScene(indices,working_cloud,observation_data,self.roi_filter,offline_data)

        # store the root scene so we can align future clouds in reference to it
        if(self.root_scene is None):
            self.root_scene = new_scene

        rospy.loginfo("new scene added, with " + str(new_scene.num_segments) + " segments")
        rospy.loginfo("Applying tracking and filtering to see how many of these are interesting")

        if(self.cur_scene):
            self.prev_scene = self.cur_scene

        self.cur_scene = new_scene

        if(self.prev_scene and self.cur_scene and self.root_scene):
            rospy.loginfo("--- Checking segments ---")
            if(len(self.cur_scene.segment_list) > 0 and len(self.prev_scene.segment_list) > 0):
                rospy.loginfo("we have a previous observation to compare to")
                tracker = OctomapSimilarityTrackerStrategy()
                tracker.track(self.cur_scene,self.prev_scene,self.root_scene)
        else:
            rospy.loginfo("no previous scene to compare to, skipping merging step, all segments regarded as new")


        return new_scene



if __name__ == '__main__':
    rospy.init_node('CT_TEST_NODE', anonymous = True)
    tracker = SegmentProcessor()

    cur_observation_data = {}
    cur_observation_data['rgb_image'] = rospy.wait_for_message("/head_xtion/rgb/image_rect_color", Image, timeout=10.0)
    cur_observation_data['depth_image'] = rospy.wait_for_message("/head_xtion/depth/image", Image, timeout=10.0)
    cur_observation_data['camera_info'] = rospy.wait_for_message("/head_xtion/depth/camera_info", CameraInfo, timeout=10.0)
    cur_observation_data['scene_cloud'] = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2,timeout=10.0)
    cur_observation_data['waypoint'] = "MY HOUSE"
    cur_observation_data['timestamp'] = int(rospy.Time.now().to_sec())
    cur_observation_data['robot_pose'] = rospy.wait_for_message("/robot_pose", Pose, timeout=10.0)
    #
    print("making listener")
    listener = TransformationStore()
    listener.create_live()
    print("waiting for listener")
    rospy.sleep(2)
    listener.kill()
    cur_observation_data['tf'] = listener.get_as_msg()

    #print(len(cur_observation_data['tf'].transforms))


    #cloud = python_pcd.read_pcd("tsc1.pcd")
    #cloud = cloud[0]
    tracker.add_unsegmented_scene(cur_observation_data)

    #invar = raw_input('press key to take view')
    #cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    #tracker.add_unsegmented_scene(cloud)
    #invar = raw_input('press key to take view')
    #cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    #tracker.add_unsegmented_scene(cloud)

    #rospy.spin()
