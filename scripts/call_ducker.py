#!/usr/bin/env python
# coding: UTF-8
import math
import time

import numpy as np

from move.msg import AmountGoal, AmountAction
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import rospy
import actionlib
from sound_system.srv import HotwordService, StringService
from std_msgs.msg import String, Int32, Bool
from nav_msgs.msg import Odometry
from ros_posenet.msg import *
from geometry_msgs.msg import Point, Quaternion
from module.rviz_marker import RvizMarker
from module.se import SE

MARGIN = 0.95


class CallDucker:
    def __init__(self):
        self.cv_image = None
        self.point_list = ["nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder", "rightShoulder"]
        self.sensor_x = 0
        self.sensor_y = 0
        self.sensor_rad = 0
        self.status = None
        self.marker = RvizMarker()
        self.sound_source_angle_list = []
        self.se = SE()
        self.failed = False
        
        rospy.init_node("raise_hand_human")
        rospy.Subscriber("/tfpose_ros/result", Poses, self.pose_callback, queue_size=1)
        rospy.Subscriber("/call_ducker/control", String, self.control_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/sound_direction", Int32, self.respeaker_callback)
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.amount_client = actionlib.SimpleActionClient("/move/amount", AmountAction)
        self.shutter_pub = rospy.Publisher("/tfpose_ros/shutter", String, queue_size=10)
        self.finish_pub = rospy.Publisher("/call_ducker/finish", Bool, queue_size=10)
    
    @staticmethod
    def to_angle(rad):
        return rad * 180 / math.pi
    
    @staticmethod
    def to_radian(angle):
        return (angle * math.pi) / 180
    
    @staticmethod
    def to_quaternion_rad(w, z):
        return math.acos(w) * 2 * np.sign(z)
    
    def send_move_base(self, point):
        # type:(tuple)->int
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point[0], point[1], point[2])
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.move_base_client.wait_for_server()
        print "move_baseに送信"
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_state()
    
    @staticmethod
    def speak(sentence):
        # type: (str) -> None
        """
        発話関数
        :param sentence:
        :return:
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(sentence)
    
    @staticmethod
    def hot_word():
        """
        「hey, ducker」に反応
        :return:
        """
        rospy.wait_for_service("/sound_system/hotword", timeout=1)
        print "hot_word待機"
        rospy.ServiceProxy("/sound_system/hotword", HotwordService)()
    
    def move_turn(self, angle):
        """
        角度送信
        :param angle:
        :return:
        """
        goal = AmountGoal()
        goal.amount.angle = angle
        goal.velocity.angular_rate = 0.5
        
        self.amount_client.wait_for_server()
        self.amount_client.send_goal(goal)
        self.amount_client.wait_for_result(rospy.Duration(10))
    
    def turn_sound_source(self):
        """
        音源定位した後の移動
        :return:
        """
        # 回転メッセージを投げる
        angle = self.sound_source_angle_list[3]  # 時間を遡る
        if angle - 180 > 0:
            angle = -(360 - angle)
        self.move_turn(angle)
    
    def calc_person_position(self, pose):
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
            return distance, (ave_x, ave_y, ave_z)
        except ZeroDivisionError:
            print "countが0 @raise_hand"
        return None
    
    @staticmethod
    def is_raise_hand(key_points):
        # type: (dict)->bool
        """
        腕を上げているかの判定
        :return:
        """
        body_part_dict = {}
        for key in key_points:
            body_part_dict.setdefault(key.part, [key.position.x, key.position.y, key.position.z])
        
        if "rightShoulder" in body_part_dict:
            if "rightElbow" in body_part_dict:
                if body_part_dict["rightShoulder"][1] > body_part_dict["rightElbow"][1]:
                    return True
            if "rightWrist" in body_part_dict:
                if body_part_dict["rightShoulder"][1] > body_part_dict["rightWrist"][1]:
                    return True
        
        if "leftShoulder" in body_part_dict:
            if "leftElbow" in body_part_dict:
                if body_part_dict["leftShoulder"][1] > body_part_dict["leftElbow"][1]:
                    return True
            if "leftWrist" in body_part_dict:
                if body_part_dict["leftShoulder"][1] > body_part_dict["leftWrist"][1]:
                    return True
        
        return False
    
    def calc_real_position(self, point):
        # type:(tuple)->tuple
        relative_theta = self.sensor_rad
        relative_x = point[2]
        relative_y = -point[0]
        
        x = (relative_x * math.cos(relative_theta) - relative_y * math.sin(relative_theta)) + self.sensor_x
        y = (relative_x * math.sin(relative_theta) + relative_y * math.cos(relative_theta)) + self.sensor_y
        print "real", x, y
        return x, y
    
    @staticmethod
    def calc_safe_position(margin, person_position):
        # type: (float,tuple)->tuple
        """
        人間を中心にdistanceを半径とした円と、人間からロボットまで結んだ直線の交点を計算
        :param margin:
        :param person_position:
        :return:
        """
        real_person_x = person_position[0]
        real_person_y = person_position[1]
        """
        distance = math.sqrt((real_person_x - self.sensor_x) ** 2 + (real_person_y - self.sensor_y) ** 2)
        # 三角形の相似を利用して算出
        safety_person_x = real_person_x - (margin * abs(real_person_x - self.sensor_x)) / distance
        safety_person_y = real_person_y - (margin * (real_person_y - self.sensor_y)) / distance
        """
        safety_person_x = real_person_x - margin
        safety_person_y = real_person_y
        
        return safety_person_x, safety_person_y
    
    ################################################################################
    def respeaker_callback(self, msg):
        # type:(Int32)->None
        """
        angle_listにスタック
        :param msg:
        :return:
        """
        angle = msg.data
        if len(self.sound_source_angle_list) > 10:
            self.sound_source_angle_list.pop(0)
        self.sound_source_angle_list.append(angle)
    
    def control_callback(self, msg):
        # type:(String)->None
        if msg.data == "start":
            for i in range(5):
                self.status = None
                self.failed = False
                # 音源定位
                self.hot_word()
                self.turn_sound_source()
                self.speak("Please raise your hand.")
                self.shutter_pub.publish(String(data="take"))
                time.sleep(3)
                print "シャッター"
                while self.status is None and not self.failed:
                    print self.status
                    print self.failed
                    pass
                if self.status is None:
                    self.speak("I am sorry, not found you. Please call, 'Hey, Ducker.' again.")
                    continue
                if self.status == actionlib.GoalStatus.SUCCEEDED:
                    self.finish_pub.publish(Bool(data=True))
                    return
                else:
                    self.speak("The destination could not be set. Please call, 'Hey, Ducker.' again.")
            self.finish_pub.publish(Bool(data=False))
    
    def odometry_callback(self, msg):
        # type: (Odometry)->None
        """
        位置情報の受け取り
        :param msg:
        :return:
        """
        self.sensor_x = msg.pose.pose.position.x
        self.sensor_y = msg.pose.pose.position.y
        self.sensor_rad = self.to_quaternion_rad(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
    
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
            
            result = self.calc_person_position(pose)
            if result is None:
                continue
            person_position.setdefault(result[0], result[1])
        
        # 手を上げている一番近い人間
        if len(person_position) == 0:
            print "誰もいない"
            self.failed = True
            return
        if min(person_position) < MARGIN:
            print "人間が近すぎます"
            self.failed = True
            return
        
        print "発見"
        self.se.play(self.se.DISCOVERY)
        real_position = self.calc_real_position(person_position[min(person_position)])
        result = self.calc_safe_position(MARGIN, (real_position[0], real_position[1]))
        self.status = self.send_move_base((result[0], result[1], 0))
        if self.status == actionlib.GoalStatus.SUCCEEDED:
            print "到着"
        else:
            print"失敗"
            self.failed = True


if __name__ == '__main__':
    CallDucker()
    rospy.spin()
