import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
# recog stuff
from recognition_srv_definitions.srv import *


if __name__ == '__main__':
    rospy.init_node("stuff")
    try:
        rospy.loginfo("getting recognition service")
        rospy.wait_for_service('/recognition_service/sv_recognition',20)
        recog_service = rospy.ServiceProxy('/recognition_service/sv_recognition',recognize)
        rospy.loginfo("Got the recognition service!")
    except Exception,e:
        rospy.loginfo("Unable to get object recognition service, continuing but no object recognition will be performed")


    try:
        rospy.loginfo("looking for recognition service")
        cl = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
        recog_out = recog_service(cloud=cl)

        print(labels)
        print(confidences)

    except Exception, e:
        rospy.logwarn("Couldn't run recognition service, or service not online")
        rospy.logwarn(e)
