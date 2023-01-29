#!/usr/bin/env python
from jsk_topic_tools import ConnectionBasedTransport
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import rospy

class PathToPoseArray(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pub = self.advertise('~output', PoseArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Path, self.cb)

    def unsubscribe(self):
        self.sub.unregister()

    def cb(self, msg):
        pub_msg = PoseArray()
        pub_msg.header = msg.header
        poses = []
        
        pose_stampeds = msg.poses
        for pose_stamped in pose_stampeds:
            poses.append(pose_stamped.pose)

        pub_msg.poses = poses
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('path_to_pose_array')
    PathToPoseArray()
    rospy.spin()
