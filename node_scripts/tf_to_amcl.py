#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped


def cb(event):
    stamp = rospy.Time.now()
    try:
        # listener.waitForTransform(src_frame, dst_frame, stamp,
        #                         timeout=rospy.Duration(1)) 
       dst_pose = tfBuffer.lookup_transform(
           src_frame, dst_frame,
           stamp, timeout=rospy.Duration(1))
    except Exception as e:
        rospy.logerr(e)
        return

    # dst_pose = listener.lookupTransform(src_frame, dst_frame, stamp)

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = src_frame
    pose_msg.header.stamp = stamp
    pose_msg.pose.pose.position.x = dst_pose.transform.translation.x
    pose_msg.pose.pose.position.y = dst_pose.transform.translation.y
    pose_msg.pose.pose.position.z = dst_pose.transform.translation.z
    pose_msg.pose.pose.orientation.x = dst_pose.transform.rotation.x
    pose_msg.pose.pose.orientation.y = dst_pose.transform.rotation.y
    pose_msg.pose.pose.orientation.z = dst_pose.transform.rotation.z
    pose_msg.pose.pose.orientation.w = dst_pose.transform.rotation.w

    pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('tf_to_pose')
    pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
    src_frame = 'map'
    dst_frame = 'base_link'
    rate = rospy.get_param('~rate', 1.)
    # listener = tf.TransformListener()
    tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(3))
    listener = tf2_ros.TransformListener(tfBuffer)
    timer = rospy.Timer(rospy.Duration(1.0 / rate), cb)
    rospy.spin()
