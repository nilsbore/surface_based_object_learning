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
import python_pcd
import tf
# reg stuff #
from observation_registration_services.srv import *
from std_srvs.srv import Trigger


if __name__ == '__main__':
    rospy.init_node('BATCH_TEST', anonymous = False)
    print("waiting for /head_xtion/depth_registered/camera_info")

    camera_msg = None

    try:
        camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/butts",  CameraInfo, timeout=1)
    except Exception,e:
        print("couldn't find ")

    if(camera_msg):
        print("got it")
    else:
        print("didn't get it")

    try:
        camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=1)
    except Exception,e:
        print("couldn't find ")

    if(camera_msg):
        print("got it")
    else:
        print("didn't get it")
