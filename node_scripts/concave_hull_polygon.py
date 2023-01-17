#!/usr/bin/env python

from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import PolygonStamped, Point32
import rospy
import alphashape
from shapely.geometry.base import dump_coords
import numpy as np
import PyKDL
import tf2_geometry_msgs
import tf2_ros

class ConcaveHullPolygon(object):

    def __init__(self):
        self.n_input = rospy.get_param('~number_of_input', 2)
        self.n_input = int(self.n_input)
        if self.n_input <= 0:
            rospy.logerr('~number_of_input should be greater than 0.')
            sys.exit(1)
	self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.pub = rospy.Publisher('~output', PolygonStamped, queue_size=1)
        self.subs = {}
        self.data = {}
        self._duration_timeout = rospy.get_param("~timeout", 6.0)
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.subscribe()
        rate = rospy.get_param('~rate', 100)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 100.')
            rate = 100
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def subscribe(self):
        for i in range(self.n_input):
            topic_name = '~input{}'.format(i + 1)
            topic_name = rospy.resolve_name(topic_name)
            sub = rospy.Subscriber(topic_name, PolygonArray,
                                   callback=lambda msg, tn=topic_name: self.callback(tn, msg),
                                   queue_size=1)
            self.subs[topic_name] = sub

    def callback(self, topic_name, msg):
        points = []
        for polygon_stamped in msg.polygons:
            try:
                pykdl_transform = tf2_geometry_msgs.transform_to_kdl(
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

            for point in polygon_stamped.polygon.points:
                x, y, z = pykdl_transform * PyKDL.Vector(
                    point.x, point.y, 0)
                points.append([x, y])
        self.data[topic_name] = points

    def timer_cb(self, timer):
        if len(self.data) != self.n_input:
            return
        pub_msg = PolygonStamped()
        total_points = []
        for i in self.data:
            total_points.extend(self.data[i])
        alpha_shape = alphashape.alphashape(total_points, 2.0)
        vertices = dump_coords(alpha_shape)
        vertices = np.concatenate(vertices).tolist()
        for i in range(len(vertices)//2):
            p = Point32(x=vertices[2*i], y=vertices[2*i+1], z=0.0)
            pub_msg.polygon.points.append(p)
        pub_msg.header.frame_id = self.frame_id
        pub_msg.header.stamp = rospy.Time.now()
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('concave_hull_polygon')
    ConcaveHullPolygon()
    rospy.spin()
    
