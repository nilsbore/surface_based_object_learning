import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField


def cb(data):
    print("woo")

if __name__ == '__main__':
    rospy.init_node("stuff")
    print("waiting for ccld")
    ccld = rospy.Subscriber("/pcl_segmentation_service/segmented_cloud_colored", PointCloud2, cb)
    print("got it")
    rospy.spin()
