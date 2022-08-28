#!/usr/bin/env python

import numpy as np
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import CameraInfo
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import PoseArray, PolygonStamped, Point32

class PersonPoseArrayToPolygonArray(ConnectionBasedTransport):

    def __init__(self):
        super(PersonPoseArrayToPolygonArray, self).__init__()
        self.frame_id = rospy.get_param("~frame_id", 'base_link')

	self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._arm = rospy.get_param("~target_arm", "larm")
        self._duration_timeout = rospy.get_param("~timeout", 3.0)
        self.padding = rospy.get_param("~padding", 0.2)
        self.pub = self.advertise('~output', PolygonArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PoseArray, self._cb_pose)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb_pose(self, msg):
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
            if y * (-1 if self._arm == "larm" else 1) > 0:
                continue
            if np.sqrt(x ** 2 + y ** 2) > 1.0:
                continue
            position = (x, y, z)
            break

        polygon_array_msg = PolygonArray()
        polygon_array_msg.header = msg.header
        polygon_array_msg.header.frame_id = self.frame_id

        if position:
            p0 = Point32(x=position[0] - self.padding, y=position[1] - self.padding, z=0)
            p1 = Point32(x=position[0] - self.padding, y=position[1] + self.padding, z=0)
            p2 = Point32(x=position[0] + self.padding, y=position[1] + self.padding, z=0)
            p3 = Point32(x=position[0] + self.padding, y=position[1] - self.padding, z=0)
            polygon_stamped_msg = PolygonStamped()
            polygon_stamped_msg.header.frame_id = self.frame_id
            polygon_stamped_msg.polygon.points = [p0, p1, p2, p3]
            polygon_array_msg.polygons.append(polygon_stamped_msg)

        self.pub.publish(polygon_array_msg)

if __name__ == '__main__':
    rospy.init_node('person_pose_array_to_polygon_array')
    PersonPoseArrayToPolygonArray()
    rospy.spin()
