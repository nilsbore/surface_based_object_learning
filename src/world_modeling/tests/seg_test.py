import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from recognition_srv_definitions.srv import *
import python_pcd
from segmentation_srv_definitions.srv import *
import sensor_msgs.point_cloud2 as pc2

def cloud_callback(data):
    print("got pc")
    python_pcd.write_pcd("VIENNA_tsc1.pcd",data)
    print("done")

if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)
    #print("waiting for pc topic")

    testcl = python_pcd.read_pcd("../../../soma_seg/tests/tsc1.pcd")
    testcl = testcl[0]
    o_sub = rospy.Subscriber("/pcl_segmentation_service/segmented_cloud_colored", PointCloud2, cloud_callback)
    rospy.sleep(1)

    #testcl = rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #rec_service = rospy.ServiceProxy("/recognition_service/sv_recognition", recognize)
    seg_service = rospy.ServiceProxy("/pcl_segmentation_service/pcl_segmentation", segment)
    print("segmenting")
    seg_service(cloud=testcl)

    print("done")
    rospy.spin()
