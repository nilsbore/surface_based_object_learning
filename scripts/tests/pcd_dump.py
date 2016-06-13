import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO

if __name__ == '__main__':
    print("connecting to mongodb")
    msg_store = MessageStoreProxy()
    print("done")
