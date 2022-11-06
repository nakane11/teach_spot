#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import PoseArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros

class PersonPoseArrayToBoxes(ConnectionBasedTransport):

    def __init__(self):
        super(PersonPoseArrayToBoxes, self).__init__()
        self._arm = rospy.get_param("~target_arm", "larm")
        self._duration_timeout = rospy.get_param("~timeout", 3.0)
        self._x_offset = rospy.get_param("~x_offset", 0.2)

	self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        rospy.loginfo("target frame_id: {}".format(self.base_frame_id))
        self.pub = self.advertise('~output', BoundingBoxArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PoseArray, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        try:
            pykdl_transform_base_to_laser = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self.base_frame_id,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rospy.Duration(self._duration_timeout)))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        pose_array = []
        for person in msg.poses:
            x, y, z = person.position.x, person.position.y, person.position.z
            x, y, z = pykdl_transform_base_to_laser * PyKDL.Vector(
                x, y, z)
            if y * (1 if self._arm == "larm" else -1) > 0:
                continue
            if np.sqrt(x ** 2 + y ** 2) > 1.0:
                continue
            pose_array.append([x, y, z])
        
        header = deepcopy(msg.header)
        header.frame_id = self.base_frame_id
        bbox_array_msg = BoundingBoxArray(header=header)

        for pose in pose_array:
            bbox_msg = BoundingBox(header=header)
            bbox_msg.pose.position.x = pose[0] + self._x_offset
            bbox_msg.pose.position.y = pose[1]
            bbox_msg.pose.position.z = 1.0
            bbox_msg.pose.orientation.x = 0
            bbox_msg.pose.orientation.y = 0
            bbox_msg.pose.orientation.z = 0
            bbox_msg.pose.orientation.w = 1
            bbox_msg.dimensions.x = 0.7
            bbox_msg.dimensions.y = 0.7
            bbox_msg.dimensions.z = 2.0
            bbox_array_msg.boxes.append(bbox_msg)

        self.pub.publish(bbox_array_msg)

if __name__ == '__main__':
    rospy.init_node('person_pose_array_to_boxes')
    PersonPoseArrayToBoxes()
    rospy.spin()
