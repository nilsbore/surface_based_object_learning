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
import os

if __name__ == '__main__':
    rospy.init_node('test_masks', anonymous = False)
    print("getting soma service")
    rospy.wait_for_service('soma2/query_db')
    print("done")
    soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
    print("making query")

    query = SOMA2QueryObjsRequest()
    query.query_type = 0
    query.objecttypes=['unknown']

    response = soma_query(query)

    if not os.path.exists("observations/"):
        os.makedirs("observations/")

    if not response.objects:
        print("No SOMA objects!")
    else:
        print("got " + str(len(response.objects)) + " SOMA objects")
        world_model = World(server_host='localhost',server_port=62345)
        for x in response.objects:
            print(x.id)
            if(x.type == "unknown"):
                wo = world_model.get_object(x.id)
                fo = wo._observations[0]
                ma = fo.get_message("rgb_mask")
                rg = fo.get_message("/head_xtion/rgb/image_rect_color")
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(ma, desired_encoding="bgr8")

                directory = "observations/"+x.id+"/"
                if not os.path.exists(directory):
                    os.makedirs(directory)
                success = cv2.imwrite(directory+'mask.jpeg',cv_image)

                cv_image = bridge.imgmsg_to_cv2(rg, desired_encoding="bgr8")
                success = cv2.imwrite(directory+'rgb.jpeg',cv_image)

                if(success):
                    print("mask file written")

    print("done")
