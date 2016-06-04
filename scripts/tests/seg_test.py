import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from recognition_srv_definitions.srv import *
import python_pcd
from segmentation_srv_definitions.srv import *
import sensor_msgs.point_cloud2 as pc2


if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)
    #print("waiting for pc topic")

    testcl = python_pcd.read_pcd("tsc1.pcd")
    testcl = testcl[0]

    #testcl = rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)
    seg_service = rospy.ServiceProxy("/pcl_segmentation_service/pcl_segmentation", segment)
    print("segmenting")
    seg_service(cloud=testcl)
    print("done")
