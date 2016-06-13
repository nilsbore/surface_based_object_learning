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
from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion
import sensor_msgs.point_cloud2 as pc2
#import python_pcd
import tf
# reg stuff #
from observation_registration_services.srv import *
import PyKDL
import tf2_ros
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class ROIFilter:

    def __init__(self):
        #rospy.init_node('world_modeling_view_alignment', anonymous = True)
        rospy.loginfo("---created ROI filter ---")
        rospy.loginfo("getting soma service")
        rospy.wait_for_service('soma2/query_db')
        rospy.loginfo("done")
        rospy.loginfo("setting up proxy")
        self.soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
        rospy.loginfo("done")
        self.gather_rois()


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

        rospy.loginfo("ROI Filter has located %d regions of interest",len(self.soma_polygons))

    def filter_object_height(self,cloud):
        min_y = 9000
        max_y = -9000
        height = 0
        for point in pc2.read_points(cloud):
            if(point[2] < min_y):
                min_y = point[2]

            if(point[2] > max_y):
                max_y = point[2]
        height = abs(max_y-min_y)


    def point_in_roi(self,point_in):

        # allows the system to deal with changes made to ROIs online
        # and avoid having to be reloaded
        self.gather_rois()

        point = Point(point_in[0],point_in[1])
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                return True
        return True


if __name__ == '__main__':
    rospy.loginfo("hi")
