import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField



def cloud_callback(data):
    print("got pc")
    pub = rospy.Publisher('/world_observations', PointCloud2, queue_size=1)
    print("republishing")
    out = pub.publish(data)
    print("unregistering")
    #o_sub.unregister()
    print("done")

if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)
    print("waiting for pc topic")
    rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    print("got it")

    # callback chain to deal with storing *objects*
    o_sub = rospy.Subscriber("/head_xtion/depth_registered/points", PointCloud2, cloud_callback)
    rospy.spin()
