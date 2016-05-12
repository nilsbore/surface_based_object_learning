import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import tf
import PyKDL
import tf2_ros


if __name__ == '__main__':
    rospy.init_node("stuff")


    cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    print("cloud header:")
    print(cloud.header)
    listener = tf.TransformListener()
    rospy.sleep(1)

    if("head_xtion_rgb_optical_frame" in str(cloud.header.frame_id)):
        print("found head_xtion_rgb_optical_frame")
        root_camera_frame = cloud.header.frame_id
        child_camera_frame = "head_xtion_rgb_frame"

    if("head_xtion_depth_optical_frame" in str(cloud.header.frame_id)):
        print("found head_xtion_depth_optical_frame")
        root_camera_frame = cloud.header.frame_id
        child_camera_frame = "head_xtion_depth_frame"

    print("frames are:")
    print(root_camera_frame)
    print(child_camera_frame)

    print("waiting for transform")
    try:
        t = listener.getLatestCommonTime("map", root_camera_frame)
        trs = listener.waitForTransform("map", root_camera_frame, t, rospy.Duration(10.0))
        print(trs)
        print(t)
    except rospy.ServiceException, e:
        print("Unable to find transform between camera frame and map frame")
        print(e)
