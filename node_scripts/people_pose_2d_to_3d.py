#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function

import numpy as np

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import Segment

class PeoplePose2Dto3D(ConnectionBasedTransport):

    limb_sequence = [[1,3],[3,5],[5,7],[7,9],[9,11],
                     [1,2],[2,4],[4,6],[6,8],[8,10],
                     [7,13],[13,15],[15,17],
                     [6,12],[12,14],[14,16],
                     [7,6],[13,12]]

    index2limbname = ["nose",
                      "left eye",
                      "right eye",
                      "left ear",
                      "right ear",
                      "left shoulder",
                      "right shoulder",
                      "left elbow",
                      "right elbow",
                      "left wrist",
                      "right wrist",
                      "left hip",
                      "right hip",
                      "left knee",
                      "right knee",
                      "left ankle",
                      "right ankle"]

    # index2limbname = ["Nose",
    #                   "Neck",
    #                   "RShoulder",
    #                   "RElbow",
    #                   "RWrist",
    #                   "LShoulder",
    #                   "LElbow",
    #                   "LWrist",
    #                   "RHip",
    #                   "RKnee",
    #                   "RAnkle",
    #                   "LHip",
    #                   "LKnee",
    #                   "LAnkle",
    #                   "REye",
    #                   "LEye",
    #                   "REar",
    #                   "LEar",
    #                   "Bkg"]
    
    def __init__(self):
        super(self.__class__, self).__init__()
        self.sub_info = None
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)
        self.pose_pub = self.advertise(
            '~output/pose', PeoplePoseArray, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_pose = message_filters.Subscriber(
            '~input/pose', PeoplePoseArray, queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            '~input/depth', Image, queue_size=1, buff_size=2**24)
        self.subs = [sub_pose, sub_depth]

        # NOTE: Camera info is not synchronized by default.
        # See https://github.com/jsk-ros-pkg/jsk_recognition/issues/2165
        sync_cam_info = rospy.get_param("~sync_camera_info", False)
        if sync_cam_info:
            sub_info = message_filters.Subscriber(
                '~input/info', CameraInfo, queue_size=1, buff_size=2**24)
            self.subs.append(sub_info)
        else:
            self.sub_info = rospy.Subscriber(
                '~input/info', CameraInfo, self._cb_cam_info)

        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        if sync_cam_info:
            sync.registerCallback(self._cb_with_depth_info)
        else:
            self.camera_info_msg = None
            sync.registerCallback(self._cb_with_depth)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()
        if self.sub_info is not None:
            self.sub_info.unregister()
            self.sub_info = None

    def _cb_cam_info(self, msg):
        self.camera_info_msg = msg
        self.sub_info.unregister()
        self.sub_info = None
        rospy.loginfo("Received camera info")

    def _cb_with_depth(self, pose_2d_array_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(
            pose_2d_array_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(
            self, pose_2d_array_msg, depth_msg, camera_info_msg
    ):
        br = cv_bridge.CvBridge()
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        skeleton_array_msg = HumanSkeletonArray()
        skeleton_array_msg.header = pose_2d_array_msg.header
        pose_3d_array_msg = PeoplePoseArray()
        pose_3d_array_msg.header = pose_2d_array_msg.header

        # calculate xyz-position
        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]

        for pose_2d_msg in pose_2d_array_msg.poses:
            limb_names = pose_2d_msg.limb_names
            scores = pose_2d_msg.scores
            poses = pose_2d_msg.poses
            skeleton_msg = HumanSkeleton()
            pose_3d_msg = PeoplePose()
            for limb_name, score, pose in zip(limb_names, scores, poses):
                position = pose.position
                if score < 0:
                    continue
                if 0 <= position.y < depth_img.shape[0] and\
                   0 <= position.x < depth_img.shape[1]:
                    z = float(depth_img[int(position.y)][int(position.x)])
                else:
                    continue
                if np.isnan(z):
                    continue
                x = (position.x - cx) * z / fx
                y = (position.y - cy) * z / fy
                pose_3d_msg.limb_names.append(limb_name)
                pose_3d_msg.scores.append(score)
                pose_3d_msg.poses.append(
                    Pose(position=Point(x=x, y=y, z=z),
                         orientation=Quaternion(w=1)))
            pose_3d_array_msg.poses.append(pose_3d_msg)

            for i, conn in enumerate(self.limb_sequence):
                j1_name = self.index2limbname[conn[0] - 1]
                j2_name = self.index2limbname[conn[1] - 1]
                if j1_name not in pose_3d_msg.limb_names \
                        or j2_name not in pose_3d_msg.limb_names:
                    continue
                j1_index = pose_3d_msg.limb_names.index(j1_name)
                j2_index = pose_3d_msg.limb_names.index(j2_name)
                bone_name = '{}->{}'.format(j1_name, j2_name)
                bone = Segment(
                    start_point=pose_3d_msg.poses[j1_index].position,
                    end_point=pose_3d_msg.poses[j2_index].position)
                skeleton_msg.bones.append(bone)
                skeleton_msg.bone_names.append(bone_name)
            skeleton_array_msg.skeletons.append(skeleton_msg)

        self.skeleton_pub.publish(skeleton_array_msg)
        self.pose_pub.publish(pose_3d_array_msg)


if __name__ == '__main__':
    rospy.init_node('people_pose_2d_to_3d')
    PeoplePose2Dto3D()
    rospy.spin()
