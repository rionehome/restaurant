# !/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from ros_posenet.msg import *
from sensor_msgs.msg import Image


class RaiseHand:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.point_list = ["nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder", "rightShoulder"]

        rospy.init_node("raise_hand_human")
        rospy.Subscriber("/ros_posenet/result", Poses, self.pose_callback)
        rospy.Subscriber("/posenet/input", Image, self.image_callback)
        self.raise_hand_position_pub = rospy.Publisher('/restaurant/raise_hand_position', String, queue_size=10)

    def pose_callback(self, msgs):
        # type: (Poses)->None
        person_position = {}
        for pose in msgs.poses:

            if not self.is_raise_hand(pose.keypoints):
                continue

            try:
                # 人間の三次元座標を計算
                count = 0
                sum_x = 0
                sum_y = 0
                sum_z = 0
                for key in pose.keypoints:
                    if key.part in self.point_list:
                        sum_x += key.position.x
                        sum_y += key.position.y
                        sum_z += key.position.z
                        count += 1
                ave_x = sum_x / count
                ave_y = sum_y / count
                ave_z = sum_z / count
                distance = math.sqrt(ave_x ** 2 + ave_y ** 2 + ave_z ** 2)
                person_position.setdefault(distance, [ave_x, ave_y, ave_z])
            except ZeroDivisionError:
                print "countが0 @raise_hand"
                return

        # 手を上げている一番近い人間
        if len(person_position) == 0:
            return
        print min(person_position), person_position[min(person_position)]

    def image_callback(self, msgs):
        # type: (Image)->None
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msgs, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

    @staticmethod
    def is_raise_hand(keypoints):
        # type: (dict)->bool
        """
        腕を上げているかの判定
        :return:
        """
        __body_part_dict__ = {}
        for key in keypoints:
            __body_part_dict__.setdefault(key.part, [key.position.x, key.position.y, key.position.z])

        if "rightElbow" in __body_part_dict__ and "rightWrist" in __body_part_dict__:
            if __body_part_dict__["rightElbow"][1] > __body_part_dict__["rightWrist"][1]:
                return True

        if "leftElbow" in __body_part_dict__ and "leftWrist" in __body_part_dict__:
            if __body_part_dict__["leftElbow"][1] > __body_part_dict__["leftWrist"][1]:
                return True
        return False


if __name__ == '__main__':
    RaiseHand()
    rospy.spin()
