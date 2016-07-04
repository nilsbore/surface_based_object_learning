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
    rospy.init_node('test_roi_service', anonymous = True)
    rospy.loginfo("Opening ROI Filter")
    r = ROIFilter()
    rospy.loginfo("Done")
