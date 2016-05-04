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

if __name__ == '__main__':
    rospy.init_node('test_soma2', anonymous = False)
    print("off we go!")

    print("getting soma service")
    rospy.wait_for_service('soma2/query_db')
    print("done")
    print("setting up proxy")
    soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
    print("done")
    print("making query")

    query = SOMA2QueryObjsRequest()
    query.query_type = 0
    query.usetimestep = False
    query.uselowertime =  False
    query.useuppertime =  False
    query.usedates =  False
    query.useweekday =  False
    query.useroi =  False

    query.objectids = (["62e1baff-750a-4108-8ee4-d82c133f3191"])
    query.objecttypes=['']

    response = soma_query(query)

    if not response.objects:
        print("empty!")
    else:
        print("got the object")

    world_model = World(server_host='localhost',server_port=62345)

    obj = world_model.get_object("2034ec47-6ec5-4366-954f-f85f3d0ae0c7")
    print("done")

    observations = obj._observations

    clouds = []
    tf_ = []
    time = []
    for o in observations:
        clouds.append(o.get_message('object_cloud'))
        tf_.append(o.get_message('/tf'))
        time.append(o.stamp)

    print("got: " + str(len(clouds)) + " clouds for object")

    print("getting metaroom reg service")

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

    response = reg_serv(additional_views=[t_cld])

    print(response)

    print("done")
    #rospy.spin()
