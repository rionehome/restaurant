# !/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ros_posenet.msg import *

MARGIN = 1


class RaiseHand:
    def __init__(self):
        self.cv_image = None
        self.point_list = ["nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder", "rightShoulder"]
        self.sensor_x = 0
        self.sensor_y = 0

        rospy.init_node("raise_hand_human")
        rospy.Subscriber("/ros_posenet/result", Poses, self.pose_callback)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.raise_hand_position_pub = rospy.Publisher('/restaurant/raise_hand_position', String, queue_size=10)

    def odometry_callback(self, msg):
        # type: (Odometry)->None
        """
        位置情報の受け取り
        :param msg:
        :return:
        """
        self.sensor_x = msg.pose.pose.position.x
        self.sensor_y = msg.pose.pose.position.y

    def pose_callback(self, msgs):
        # type: (Poses)->None
        """
        関節推定の結果を受け取り
        :param msgs:
        :return:
        """
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
        if min(person_position) < MARGIN:
            print "人間が近すぎます"
            return

        safety_person_position = self.calc_safe_position(MARGIN, person_position[min(person_position)])
        print safety_person_position

    @staticmethod
    def is_raise_hand(keypoints):
        # type: (dict)->bool
        """
        腕を上げているかの判定
        :return:
        """
        body_part_dict = {}
        for key in keypoints:
            body_part_dict.setdefault(key.part, [key.position.x, key.position.y, key.position.z])

        if "rightElbow" in body_part_dict and "rightWrist" in body_part_dict:
            if body_part_dict["rightElbow"][1] > body_part_dict["rightWrist"][1]:
                return True

        if "leftElbow" in body_part_dict and "leftWrist" in body_part_dict:
            if body_part_dict["leftElbow"][1] > body_part_dict["leftWrist"][1]:
                return True
        return False

    def calc_safe_position(self, margin, person_position):
        # type: (float,list)->tuple
        """
        人間を中心にdistanceを半径とした円と、人間からロボットまで結んだ直線の交点を計算
        :param margin:
        :param person_position:
        :return:
        """
        real_person_x = self.sensor_x + person_position[2]
        real_person_y = self.sensor_y + person_position[0]
        distance = math.sqrt((real_person_x - self.sensor_x) ** 2 + (real_person_y - self.sensor_y) ** 2)
        # 三角形の相似を利用して算出
        safety_person_x = (margin * (real_person_x - self.sensor_x)) / distance
        safety_person_y = (margin * (real_person_y - self.sensor_y)) / distance

        return safety_person_x, safety_person_y


if __name__ == '__main__':
    RaiseHand()
    rospy.spin()
