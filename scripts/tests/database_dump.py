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

import python_pcd
import pickle
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

    if not response.objects:
        print("No SOMA objects!")
    else:
        print("got " + str(len(response.objects)) + " SOMA objects")
        world_model = World(server_host='localhost',server_port=62345)
        episodes = []
        for x in response.objects:
            wo = world_model.get_object(x.id)

            if(wo.view_episode_id in episodes):
                continue
            else:
                episodes.append(wo.view_episode_id)

            directory = "view_episodes/"+str(wo.view_episode_id)+"/"
            if not os.path.exists(directory):
                os.makedirs(directory)

            for k in wo._observations:
                cloud = k.get_message("/head_xtion/depth_registered/points")
                rgb_img = k.get_message("/head_xtion/rgb/image_rect_color")
                camera_info = None

                try:
                    camera_info = k.get_message("/head_xtion/depth_registered/sw_registered/camera_info")
                except Exception,e:
                    print("couldn't find first topic")

                try:
                    camera_info = k.get_message("/head_xtion/depth_registered/camera_info")
                except Exception,e:
                    print("couldn't find second topic")

                if(camera_info is None):
                    print("---- unable to find either topic")

                robot_pose = k.get_message("/robot_pose")
                tf = k.get_message("/tf")

                #cl = cloud.retrieve()
                #python_pcd.write_pcd(directory+"cloud.pcd", cl)

                #cv_image = self.bridge.imgmsg_to_cv2(rgb_img, desired_encoding="passthrough")
                #cv2.imwrite(directory+'rgb.jpeg',cv_image)



                pickle.dump(rgb_img,open(directory+"image.p",wb))
                pickle.dump(cloud,open(directory+"cloud.p",wb))
                pickle.dump(tf,open(directory+"tf.p",wb))
                pickle.dump(camera_info,open(directory+"camera_info.p",wb))
                pickle.dump(robot_pose,open(directory+"robot_pose.p",wb))

    print("done")
