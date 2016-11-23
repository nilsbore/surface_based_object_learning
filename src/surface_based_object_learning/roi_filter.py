#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
# soma stuff
from soma_msgs.msg import SOMAObject,SOMAROIObject
from soma_manager.srv import *
from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion, Point
import sensor_msgs.point_cloud2 as pc2
#import python_pcd
import tf
# reg stuff #
from observation_registration_services.srv import *
import PyKDL
import tf2_ros
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import shapely.geometry
from shapely.geometry import MultiPoint
from surface_based_object_learning.srv import *


class ROIFilter:

    def __init__(self):
        #rospy.init_node('test_roi_filter_', anonymous = True)
        rospy.loginfo("---created ROI filter ---")
        rospy.loginfo("getting soma service")
        rospy.wait_for_service('soma/query_rois')
        rospy.loginfo("done")
        rospy.loginfo("setting up proxy")
        self.soma_query = rospy.ServiceProxy('soma/query_rois',SOMAQueryROIs)
        rospy.loginfo("done")
        self.gather_rois()
        rospy.loginfo("launching SOMa ROI check server")
        point_check_service = rospy.Service('/check_point_in_soma_roi',PointInROI,self.roi_check_service_cb)
        point_set_check_service = rospy.Service('/check_point_set_in_soma_roi',PointSetInROI,self.roi_set_check_service_cb)

        point_set_check_service = rospy.Service('/get_closest_roi_to_robot',GetROIClosestToRobot,self.get_closest_roi_to_robot_cb)
        rospy.loginfo("SOMa ROI check service running")
        self.gather_rois()

    def get_closest_roi_to_robot_cb(self, req):
        rospy.loginfo("got robot roi req")
        p = self.get_closest_roi_to_robot(req.pose)
        return GetROIClosestToRobotResponse(p)

    def roi_check_service_cb(self, req):
        p = self.ros_point_in_roi(req.input)
        return PointInROIResponse(p)

    def roi_set_check_service_cb(self, req):
        p = self.ros_point_set_in_roi(req.input,req.pose)
        return PointSetInROIResponse(p)

    def gather_rois(self,filter_point=None):
        rospy.loginfo("Gathering ROIs")
        query = SOMAQueryROIsRequest()
        query.returnmostrecent = True
        response = self.soma_query(query)
        #rospy.loginfo(response)

        self.soma_polygons = []

        # seems like there might be a bug with SOMA, quick workaround, just log the IDs
        visited_ids = []
        #rospy.loginfo("ROI TYPES: ")
        for roi in response.rois:
            if(roi.id not in visited_ids):
                visited_ids.append(roi.id)
                #rospy.loginfo(roi.type)
                if("NavArea" in roi.type):
                #rospy.loginfo(roi.type+" \t SKIPPING")
                    continue
                if("Human" in roi.type):
                #rospy.loginfo(roi.type+" \t SKIPPING")
                    continue

                if("Robot" in roi.type):
                    continue

                if("Atrium" in roi.type):
                    continue

                points = roi.posearray.poses

                print("found roi type:" + roi.type)

                print(roi.type)
                points_2d = []

                for point in points:
                    points_2d.append([point.position.x,point.position.y])

                if(len(points_2d) <= 2):
                    print("Found one SOMA region that isn't a polygon (doesn't have >= 3 points) so I am skipping it")
                    continue

                polygon = Polygon(points_2d)
                print(polygon)
                self.soma_polygons.append(polygon)

        if(filter_point is not None):
            filter_point = shapely.geometry.Point(filter_point.x,filter_point.y)
            rospy.loginfo("filtering by pose")
            filtered_polygons = []
            b_d = 900000
            p_d = 900000
            best_p = None
            for p in self.soma_polygons:
                ds = p.centroid.distance(filter_point)
                if(ds < p_d):
                    p_d = ds
                    best_p = p
            self.soma_polygons = [best_p]



        # TODO: Expand this later to support proper polygons
    def get_closest_roi_to_robot(self,filter_point):
        rospy.loginfo("executing robot roi req")
        self.gather_rois(filter_point)
        poly = self.soma_polygons[0]
        pa = geometry_msgs.msg.PoseArray()
        min_x = poly.bounds[0]
        min_y = poly.bounds[1]
        max_x = poly.bounds[2]
        max_y = poly.bounds[3]

        mn = geometry_msgs.msg.Pose()
        mn.position.x = min_x
        mn.position.y = min_y

        mx = geometry_msgs.msg.Pose()
        mx.position.x = max_x
        mx.position.y = max_y

        pa.poses.append(mn)
        pa.poses.append(mx)
        rospy.loginfo("returning pa:")
        rospy.loginfo(pa)
        return pa





    def get_points_in_rois(self,point_set):
        self.gather_rois()
        ret = []
        for k in point_set:
            for polygon in self.soma_polygons:
                if(polygon.contains(point)):
                    ret.append(k)
        return ret

    def ros_point_in_roi(self,point_in):
        # allows the system to deal with changes made to ROIs online
        # and avoid having to be reloaded
        self.gather_rois()
        #rospy.loginfo("Checking: " + str(len(self.soma_polygons)) + " SOMa ROIs")
        point = shapely.geometry.Point(point_in.x,point_in.y)
        for poly in self.soma_polygons:
            if(poly.contains(point)):
                #rospy.loginfo("IN ROI:" + str(point))
                return True
        #rospy.loginfo("OUTSIDE ROI:" + str(point))
        return False

    def ros_point_set_in_roi(self,point_in,filter_point):
        # allows the system to deal with changes made to ROIs online
        # and avoid having to be reloaded
        self.gather_rois(filter_point)
        output = []
        for p in point_in:
            point = shapely.geometry.Point(p.x,p.y)
            for poly in self.soma_polygons:
                if(poly.contains(point)):
                    output.append(True)
                else:
                    output.append(False)
        print("done, processed " + str(len(point_in)) + " points")
        return output

    def point_in_roi(self,point_in):
        # allows the system to deal with changes made to ROIs online
        # and avoid having to be reloaded
        self.gather_rois()
        point = shapely.geometry.Point(point_in[0],point_in[1])
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                return True
        return False

    def accel_point_in_roi(self,point_in):
        point = shapely.geometry.Point(point_in[0],point_in[1])
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                #rospy.loginfo("IN ROI:" + str(point))
                return True
        #rospy.loginfo("OUTSIDE ROI:" + str(point))
        return False

    def accel_any_point_in_roi(self,cloud_in):

        for k in cloud_in:
            point = shapely.geometry.Point(k[0],k[1])
            for polygon in self.soma_polygons:
                if(polygon.contains(point)):
                    return True
        return False

    def filter_full_cloud(self,cloud,tf=None):
        filtered_points = []
        self.gather_rois()
        rospy.loginfo("filtering full cloud")
        #print(cloud.fields)
        #python_pcd.write_pcd("input_points.pcd", cloud, overwrite=True)
        mc = cloud
        if(tf is not None):
            mc = self.transform_frame_to_map(cloud,tf)

        map_int_data = list(pc2.read_points(mc))
        orig_int_data = list(pc2.read_points(cloud))

        for m,o in zip(map_int_data,orig_int_data):
            point = shapely.geometry.Point(m[0],m[1])
            for polygon in self.soma_polygons:
                if(polygon.contains(point)):
                    filtered_points.append(o)

        rospy.loginfo("done")
        rospy.loginfo("input cloud size: " + str(len(map_int_data)))
        rospy.loginfo("output cloud size: " + str(len(filtered_points)))

        return pc2.create_cloud(cloud.header,cloud.fields,filtered_points)


    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))

    def transform_frame_to_map(self,cloud,tf):
        rospy.loginfo("creating transformer")

        if(tf is None):
            self.listener = tf.TransformListener()
        else:
            self.listener = TransformationStore().msg_to_transformer(tf)


        rospy.sleep(5)
        self.child_camera_frame = cloud.header.frame_id
        rospy.loginfo("doing conversion")
        t = self.listener.getLatestCommonTime("map", self.child_camera_frame)
        tr_r = self.listener.lookupTransform("map", self.child_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
        #self.transform_frame_to_map = tr

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = 'map'
        tr_s.child_frame_id = self.child_camera_frame
        tr_s.transform = tr


        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        fil_fields = []
        for x in cloud.fields:
            if(x.name in "x" or x.name in "y" or x.name in "z" or x.name in "rgb"):
                fil_fields.append(x)

        res = pc2.create_cloud(cloud.header, fil_fields, points_out)
        return res

if __name__ == '__main__':
    rospy.init_node('test_roi_filter_', anonymous = True)
    rospy.loginfo("loading ROI Filter")
    r = ROIFilter()
    print("getting pose closest to robot")
    pose = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)
    print("position:")
    print(pose)
    roi = r.get_closest_roi_to_robot(pose.position)
    print("done")
    print(roi)
