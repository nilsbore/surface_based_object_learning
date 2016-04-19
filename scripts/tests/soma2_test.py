import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *

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

    query.objectids = (["hello"])
    query.objecttypes=['']

    response = soma_query(query)
    print(response)

    if not response.objects:
        print("empty!")

    print(response.objects[0])


    print("done")

    #print("trying to insert")
    #soma_insert = rospy.ServiceProxy('soma2/insert_objects',SOMA2InsertObjs)
    #print("getting service")
    #rospy.wait_for_service('soma2/insert_objects')
    #print("done")

    #new_obj = SOMA2Object()
    #new_obj.id = "hello"

    #print("object created")

    #soma_insert([new_obj])

    #print("object inserted")


    rospy.spin()
