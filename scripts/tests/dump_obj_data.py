import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from soma_io.observation import *
from soma_io.geometry import *
from soma_io.state import World, Object
from cv_bridge import CvBridge, CvBridgeError
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose
import cv
import cv2
from sensor_msgs.msg import PointCloud2, PointField
import python_pcd
from world_modeling.srv import *
from std_srvs.srv import Trigger

if __name__ == '__main__':
    rospy.init_node('test_soma2', anonymous = False)

    world_update = rospy.ServiceProxy('update_world_model',WorldUpdate)
    begin_obs = rospy.ServiceProxy('/begin_observations',Trigger)
    end_obs = rospy.ServiceProxy('/end_observations',Trigger)

    world_model = World(server_host='localhost',server_port=62345)

    #obj = world_model.get_object("ff9a89f9-1b91-4ea7-9e01-604f02be48fe")
    obj = world_model.get_object("11b4851d-efc2-4839-961d-322c414d22f3")
    print("done")

    bridge = CvBridge()
    observations = obj._observations

    #begin_obs()
    for o in observations:
        print("observation")
        cv_image = bridge.imgmsg_to_cv2(o.get_message("rgb_mask"),desired_encoding="passthrough")
        cv2.imwrite("mask"+str(observations.index(o))+".jpeg",cv_image)

        #pcd = o.get_message("/head_xtion/depth_registered/points")
        #world_update(input=pcd,waypoint="WayPoint32")
        #invar = raw_input('press key to take view')

    #end_obs()


    #rospy.spin()
