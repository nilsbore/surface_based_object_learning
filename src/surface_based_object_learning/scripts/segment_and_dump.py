#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from surface_based_object_learning.srv import *
from segmentation_srv_definitions.srv import *
import python_pcd
import sensor_msgs.point_cloud2 as pc2



if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
    rospy.wait_for_service('/object_gestalt_segmentation')
    print("got it")

    print("making proxy")
    seg = rospy.ServiceProxy('/object_gestalt_segmentation',segment)
    print("done")

    print("getting cloud")
    scene = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    print("segmenting")
    sc = seg(cloud=scene)
    int_data = list(pc2.read_points(scene))
    indices = sc.clusters_indices
    filtered_clusters = []
    for cluster in indices:
        print 'cluster has %f points' %len(cluster.data)
        ic = []
        for point in cluster.data:
            ic.append(int_data[point])
        filtered_clusters.append(ic)ls


    print 'filtered clusters: %d' %len(filtered_clusters)

    for k in filtered_clusters:
        print("writing file")
        cld = pc2.create_cloud(scene.header, scene.fields, k)
        python_pcd.write_pcd(str(filtered_clusters.index(k))+".pcd",cld,overwrite=True)

    #rospy.spin()
