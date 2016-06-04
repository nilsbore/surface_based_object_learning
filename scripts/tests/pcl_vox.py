import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np

if __name__ == '__main__':
    rospy.init_node('ws_repeater', anonymous = False)

    print("getting cloud")
    cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    pts = []
    for point in pc2.read_points(cloud):
        pts.append([point[0],point[1],point[2]])

    print("done")

    pts = np.array(pts,dtype=np.float32)

    pc = pcl.PointCloud()

    pc.from_array(pts)

    print("got pcl cloud")

    grid = pc.make_octree(0.1)
    grid.add_points_from_input_cloud()

    c = grid.get_occupied_voxel_centers()

    oc = grid.is_voxel_occupied_at_point(np.array((1.4367570877075195, 0.5520142912864685, 3.4150002002716064),dtype=np.float32))

    print(oc)

    #pcl.save(grid,"grid.pcd")

    #pcl.save(pc,"orig.pcd")
    #rospy.spin()
