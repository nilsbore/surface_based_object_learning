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
import python_pcd
from shapely.geometry import MultiPoint
from world_modeling.srv import PointInROI


class ROIFilter:

    def __init__(self):
        #rospy.init_node('test_roi_filter_', anonymous = True)
        rospy.loginfo("---created ROI filter ---")
        rospy.loginfo("getting soma service")
        rospy.wait_for_service('soma2/query_db')
        rospy.loginfo("done")
        rospy.loginfo("setting up proxy")
        self.soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
        rospy.loginfo("done")
        self.gather_rois()
        rospy.loginfo("launching SOMa ROI check server")
        point_check_service = rospy.Service('/check_point_in_soma_roi',PointInROI,self.roi_check_service_cb)
        rospy.loginfo("SOMa ROI check service running")

    def roi_check_service_cb(self, req):
        p = self.ros_point_in_roi(req.input)
        return PointInROIResponse(p)

    def gather_rois(self):
        query = SOMA2QueryObjsRequest()
        query.query_type = 2
        response = self.soma_query(query)

        self.soma_polygons = []

        for roi in response.rois:
            if("RobotNavArea" in roi.type):
                continue
            if("HumanWorkplace" in roi.type):
                continue

            points = roi.posearray.poses
            points_2d = []

            for point in points:
                points_2d.append([point.position.x,point.position.y])

            if(len(points_2d) <= 2):
                print("Found one SOMA region that isn't a polygon (doesn't have >= 3 points) so I am skipping it")
                continue

            polygon = Polygon(points_2d)
            self.soma_polygons.append(polygon)

        #rospy.loginfo("ROI Filter has located %d regions of interest",len(self.soma_polygons))

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
        point = shapely.geometry.Point(point_in.x,point_in.y)
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                return True
        return False

    def point_in_roi(self,point_in):
        # allows the system to deal with changes made to ROIs online
        # and avoid having to be reloaded
        self.gather_rois()
        point = shapely.geometry.Point(point_in[0],point_in[1])
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                return True
        return False

    def filter_full_cloud(self,cloud):
        filtered_points = []
        self.gather_rois()
        rospy.loginfo("filtering full cloud")
        #print(cloud.fields)
        #python_pcd.write_pcd("input_points.pcd", cloud, overwrite=True)
        mc = self.transform_frame_to_map(cloud)
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

    def transform_frame_to_map(self,cloud):
        rospy.loginfo("creating transformer")
        self.listener = tf.TransformListener()
        rospy.sleep(5)
        self.child_camera_frame = cloud.header.frame_id
        rospy.loginfo("doing conversion")
        t = self.listener.getLatestCommonTime("map", self.child_camera_frame)
        self.listener.waitForTransform("map", self.child_camera_frame, t, rospy.Duration(5.0))
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


    rospy.loginfo("getting service")
    point = geometry_msgs.msg.Point()
    point.x = 10
    point.y = 10
    point.z = 10
    service = rospy.ServiceProxy('/check_point_in_soma_roi',PointInROI)
    rospy.loginfo("done, got service")
    rospy.spin()
