#!/usr/bin/env python

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


class InfoToDepthImage(ConnectionBasedTransport):

    def __init__(self):
        super(InfoToDepthImage, self).__init__()
        self.frame_height = rospy.get_param('~frame_height', 2.0)
        self.bridge = cv_bridge.CvBridge()
        self.pub_image = self.advertise(
            '~output', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', CameraInfo, self.callback,
                                    queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, info_msg):
        bridge = self.bridge

        width, height = info_msg.width,  info_msg.height
        depth = self.frame_height * np.ones((height, width), dtype=np.float32)
        depth_msg = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = info_msg.header
        self.pub_image.publish(depth_msg)

if __name__ == '__main__':
    rospy.init_node('info_to_depth_image')
    node = InfoToDepthImage()
    rospy.spin()
