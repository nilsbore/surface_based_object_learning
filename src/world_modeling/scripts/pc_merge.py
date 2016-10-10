import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from soma_io.observation import Observation, TransformationStore
from soma_io.geometry import *
from soma_io.state import World, Object
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
import python_pcd

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

    obj = world_model.get_object("50b22f1c-c66a-4760-be54-fdf64504a3d9")
    print("done")

    observations = obj._observations

    clouds = []
    for o in observations:
        clouds.append(o.get_message('object_cloud'))

    print("got: " + str(len(clouds)) + " clouds for object")

    # ok now can we merge these into a single cloud?

    combined_cloud_points = []
    for c in clouds:
        print("combining cloud")
        raw_cloud = pc2.read_points(c)
        int_data = list(raw_cloud)

        for point in int_data:
            x = point[0]
            y = point[1]
            z = point[2]
            col = point[3]
            combined_cloud_points.append((x,y,z,col))

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    combined_cloud = pc2.create_cloud(header, clouds[0].fields, combined_cloud_points)

    print("waiting to write...")

    python_pcd.write_pcd("mug.pcd", combined_cloud)

    print("done")
    #rospy.spin()
