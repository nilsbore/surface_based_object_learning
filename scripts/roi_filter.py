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
        soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
        rospy.loginfo("done")
        rospy.loginfo("making query")
        query = SOMA2QueryObjsRequest()
        query.query_type = 2
        response = soma_query(query)

        self.soma_polygons = []

        for roi in response.rois:
            points = roi.posearray.poses
            points_2d = []
            for point in points:
                points_2d.append([point.position.x,point.position.y])
            polygon = Polygon(points_2d)
            self.soma_polygons.append(polygon)

        rospy.loginfo("ROI Filter has located %d regions of interest",len(self.soma_polygons))

    def point_in_roi(self,point_in):
        point = Point(point_in[0],point_in[1])
        for polygon in self.soma_polygons:
            if(polygon.contains(point)):
                return True
        return False


if __name__ == '__main__':
    rospy.loginfo("hi")
