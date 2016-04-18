import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from soma_io.soma_io import Importer



if __name__ == '__main__':
    importer = Importer('/world_observations')
    importer.init_subscriber('base_link','head_xtion_rgb_optical_frame','map')
    # need to do anything more here?
