import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from recognition_srv_definitions.srv import *
import python_pcd


if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)
    print("waiting for pc topic")

    testcl = python_pcd.read_pcd("cloud_00000012.pcd")
    print(testcl[0].header)
    input_cloud = rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)
    response = rec_service(cloud=testcl[0])
    print(response.ids[0])
    print(response.confidence[0])
    print(response.models_cloud[0].header)
