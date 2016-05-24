import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField


def cb(data):
    print("woo")

if __name__ == '__main__':
    rospy.init_node("stuff")
    try:
        rospy.loginfo("getting recognition service")
        rospy.wait_for_service('/recognition_service/sv_recognition',20)
        recog_service = rospy.ServiceProxy('/recognition_service/sv_recognition',recognize)
        rospy.loginfo("Got the recognition service!")
    except Exception,e:
        rospy.loginfo("Unable to get object recognition service, continuing but no object recognition will be performed")

    cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)

    try:
        rospy.loginfo("looking for recognition service")
        recog_out = self.recog_service(cloud=cur_scene_cluster.segmented_pc_mapframe)
        labels = recog_out.ids
        confidences = recog_out.confidence

        print(labels)
        print(confidences)

    except Exception, e:
        rospy.logwarn("Couldn't run recognition service, or service not online")
