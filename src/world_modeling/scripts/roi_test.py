import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy.spatial import ConvexHull
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose
from soma_io.state import World, Object
from scipy.spatial import Delaunay
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from matplotlib.path import Path


if __name__ == '__main__':
    rospy.init_node('test_masks', anonymous = False)
    print("getting soma service")
    rospy.wait_for_service('soma2/query_db')
    print("done")
    print("setting up proxy")
    soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
    print("done")
    print("making query")

    query = SOMA2QueryObjsRequest()
    query.query_type = 2

    response = soma_query(query)

    points_2d = []
    test = []
    for roi in response.rois:
        points = roi.posearray.poses
        for point in points:
            points_2d.append([point.position.x,point.position.y])
            test = [point.position.x,point.position.y]

    polygon = Polygon(points_2d)
    print(polygon)
    pt = Point(-8.78,4.11)
    print(pt)

    print(polygon.contains(pt))
