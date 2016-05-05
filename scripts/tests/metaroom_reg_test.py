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

if __name__ == '__main__':
    rospy.init_node('test_soma2', anonymous = False)
    print("off we go!")

    reg_serv = rospy.ServiceProxy('additional_view_registration_server',AdditionalViewRegistrationService)
    print("got it")

    #tr = TransformationStore.msg_to_transformer(tf[0])

    #tr_r = tr.lookupTransform("head_xtion_depth_optical_frame","odom",rospy.Time.from_sec(1461972565.708218098))
    listener = tf.TransformListener()
    listener.waitForTransform("/base_link", "/head_xtion_rgb_optical_frame", rospy.Time(), rospy.Duration(4.0))

    tr_r = listener.lookupTransform("base_link", "head_xtion_rgb_optical_frame", rospy.Time())

    transform = Transform()
    transform.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
    transform.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])

    print(transform)

    t_cld = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)

    response = reg_serv(additional_views=[t_cld,t_cld_2])

    print(response)

    print("done")
    #rospy.spin()
