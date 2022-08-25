#!/usr/bin/env python

from copy import deepcopy

import numpy as np
from geometry_msgs.msg import PoseArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from image_geometry import PinholeCameraModel

class PersonPoseArrayToMask(ConnectionBasedTransport):

    def __init__(self):
        super(PersonPoseArrayToMask, self).__init__()
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.shape = None

	self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._arm = rospy.get_param("~target_arm", "larm")
        self._duration_timeout = rospy.get_param("~timeout", 3.0)
        self.padding = rospy.get_param("~padding", 0.2)
        self.bridge = cv_bridge.CvBridge()
        self.pub_mask = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        sub_pose = rospy.Subscriber('~input', PoseArray, self._cb_pose)
        sub_info = rospy.Subscriber('~input/info', CameraInfo, self._cb_info,
                                    queue_size=1, buff_size=2**24)
        self.subs = [sub_pose, sub_info]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb_info(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.frame_id = msg.header.frame_id
        self.shape = msg.height, msg.width
        self.is_camera_arrived = True

    def _cb_pose(self, msg):
        if not self.is_camera_arrived:
            return
        try:
            pykdl_transform_cam_to_laser = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self.frame_id,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rospy.Duration(self._duration_timeout)))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        position = None
        for person in msg.poses:
            x, y, z = person.position.x, person.position.y, person.position.z
            x, y, z = pykdl_transform_cam_to_laser * PyKDL.Vector(
                x, y, z)
            if y * (1 if self._arm == "larm" else -1) > 0:
                continue
            if np.sqrt(x ** 2 + y ** 2) > 1.0:
                continue
            position = (x, y, z)
            break

        if not position:
            return

        pos_0 = (position[0] - self.padding, position[1] - self.padding, position[2])
        pos_1 = (position[0] - self.padding, position[1] + self.padding, position[2])
        pos_2 = (position[0] + self.padding, position[1] - self.padding, position[2])  
        u0, v0 = self.cameramodels.project3dToPixel(pos_0)
        u1, v1 = self.cameramodels.project3dToPixel(pos_1)
        u2, v2 = self.cameramodels.project3dToPixel(pos_2)
        mask = np.full(self.shape, 255, dtype=np.uint8)
        u0 = u0.astype(int)
        u2 = u2.astype(int)
        v0 = v0.astype(int)
        v1 = v1.astype(int)
        mask[v0:v1, u0:u2] = 0
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

if __name__ == '__main__':
    rospy.init_node('person_pose_array_to_mask')
    PersonPoseArrayToMask()
    rospy.spin()
