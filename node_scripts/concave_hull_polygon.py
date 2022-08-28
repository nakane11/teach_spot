#!/usr/bin/env python

from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import PolygonStamped
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import alphashape
from shapely.geometry.base import dump_coords

class ConcaveHullPolygon(ConnectionBasedTransport):

    def __init__(self):
        self.n_input = rospy.get_param('~number_of_input', 2)
        self.n_input = int(self.n_input)
        if self.n_input <= 0:
            rospy.logerr('~number_of_input should be greater than 0.')
            sys.exit(1)
        self.pub = self.advertise('~output', PolygonStamped, queue_size=1)
        self.subs = {}
        self.data = {}
        self.frame_id = rospy.get_param('~frame_id', None)
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

    def unsubscribe(self):
        for sub in self.subs:
            subs[sub].unregister()

    def callback(self, topic_name, msg):
        if self.frame_id is not None && msg.header.frame_id != self.frame_id:
            rospy.logwarn('frame_id of input PolygonArray should be {}'.format(self.frame_id))
            return
        points = []
        for polygon_stamped in msg.polygons:
            for point in polygon_stamped.polygon.points:
                points.append([point.x, point.y])
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
        for vertex in vertices:
            p = Point32(x=p[0], y=p[1], z=0.0)
            pub_msg.polygon.points.append(p)
        pub_msg.header.frame_id = self.frame_id
        pub_msg.header.stamp = rospy.Time.now()
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('concave_hull_polygon')
    ConcaveHullPolygons()
    rospy.spin()
    
