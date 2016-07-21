import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from cv_bridge import CvBridge, CvBridgeError
import cv2

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose
from soma_io.state import World, Object
import time
import sensor_msgs.point_cloud2 as pc2
from roi_filter import ROIFilter


if __name__ == '__main__':
    rospy.init_node('test_masks', anonymous = False)

    soma_srv_name = '/soma2/query_db'
    soma_srv = rospy.ServiceProxy(soma_srv_name, SOMA2QueryObjs)


    req = SOMA2QueryObjsRequest()
    req.query_type = 0
    req.useroi = True
    req.roi_id = "SuppLocker5"
    req.usedates = False
    #req.lowerdate = 49494 # MongoDB requires time in miliseconds
    #req.upperdate = int(time.time()) # MongoDB requires time in miliseconds

    print("lowerdate:")
    print(req.lowerdate)
    print("upperdate: ")
    print(req.upperdate)

    rospy.loginfo("Requesting objects")
    res = soma_srv(req)
    rospy.loginfo("Received objects: %s", len(res.objects))
    r = ROIFilter()
    r.gather_rois()
    for o in res.objects:
        cloud = o.cloud
        map_data = list(pc2.read_points(cloud))

        # calc centroid
        x = 0
        y = 0
        z = 0
        for p in map_data:
            x += p[0]
            y += p[1]
            z += p[2]

        x/=len(map_data)
        y/=len(map_data)
        z/=len(map_data)
        #print("CENTROID:")
        #print(x)
        #print(y)
        #print(z)
        in_roi = r.accel_any_point_in_roi(map_data)
        print(in_roi)
#        print("POSE: ")
#        print(o.pose)


    print("done")
