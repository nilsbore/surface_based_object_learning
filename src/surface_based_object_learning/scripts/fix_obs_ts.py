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
from pymongo import MongoClient
import json

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

    #if not os.path.exists("observations/"):
    #    os.makedirs("observations/")

    client = MongoClient('localhost',62345)
    print("gotcha")

    ws_db = client['world_state']
    objs_coll = ws_db['Objects']

    if not response.objects:
        print("No SOMA objects!")
    else:
        print("got " + str(len(response.objects)) + " SOMA objects")
        world_model = World(server_host='localhost',server_port=62345)
        for x in response.objects:
            print(x.id)
            if(x.type == "unknown"):
                wo = world_model.get_object(x.id)
                print("updating ts on %d observations " % len(wo._observations))
                db = objs_coll.find({'key': x.id,},snapshot=True)
                if(db):
                    print("got: " + x.id)
                    for k in db:
                        new_obs = []
                        obs = k['_observations']
                        for o in obs:
                            print(o['stamp'])
                            o['stamp'] = x.logtimestamp
                            print(o['stamp'])
                        objs_coll.save(k)


                #nobs = []
                #for o in wo._observations:
                #    print("updating")
                #    o.stamp = x.logtimestamp

                #wo._mongo_encode(wo)




                print("done")
                print("")

    print("done")
