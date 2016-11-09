#!/usr/bin/env python

import roslib
import rospy
from collections import deque
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
import tf2_ros
import tf, tf2_msgs.msg

class TransformationStore():
    """
    Subscribes to /TF, stores transforms, turns into transformer when needed
    """
    def __init__(self):
        self._transformations = deque([])
        self._lively = False
        self._max_buffer = 10

    def kill(self):
        self._sub.unregister()

    def cb(self, transforms):
        time_window = rospy.Duration(self._max_buffer)
        for transform in transforms.transforms:
            #rospy.loginfo("Got transform: %s - > %s"% ( transform.header.frame_id, transform.child_frame_id))
            if self._max_buffer > 0 and len(self._transformations) > 2:
                l =  self._transformations.popleft()
                if (transform.header.stamp -  l.header.stamp) < time_window:
                    self._transformations.appendleft(l)
            self._transformations.append(transform)

    def create_from_transforms(self, transforms):
        """
        Create a store from a given set of transforms
        transforms: Must be a list of TransformStamped messages
        """
        for t in transforms:
            self._transformations.append(t)
        return self

    def create_live(self, max_buffer=2.0):
        # subscribe to tf and store transforms
        self._max_buffer = max_buffer
        self._sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.cb)
        self._lively = True
        return self

    def get_as_msg(self):
        msg = tf2_msgs.msg.TFMessage()
        msg.transforms = self._transformations
        return msg

    def msg_to_transformer(self,msg):
        t = tf.TransformerROS()
        for transform in msg.transforms:
            t.setTransform(transform)
        return t
