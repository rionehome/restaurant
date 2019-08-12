#!/usr/bin/env python
# coding: UTF-8
import math

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import rospy
import actionlib
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ros_posenet.msg import *
from geometry_msgs.msg import Point, Quaternion
from module.rviz_marker import RvizMarker
from module.se import SE

MARGIN = 0.8


class RaiseHand:
    def __init__(self):
        self.cv_image = None
        self.point_list = ["nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder", "rightShoulder"]
        self.sensor_x = 0
        self.sensor_y = 0
        self.flag = False
        self.marker = RvizMarker()
        self.raise_hand_persons = []
        self.se = SE()
        
        rospy.init_node("raise_hand_human")
        rospy.Subscriber("/ros_posenet/result", Poses, self.pose_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.raise_hand_position_pub = rospy.Publisher('/restaurant/raise_hand_position', String, queue_size=1)
    
    def send_move_base(self, point):
        # type:(tuple)->int
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point[0], point[1], point[2])
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.client.wait_for_server()
        print "move_baseに送信"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_state()
    
    def calc_average_position(self, pose):
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
            return distance, [ave_x, ave_y, ave_z]
        except ZeroDivisionError:
            print "countが0 @raise_hand"
        return None
    
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
        
        if "rightShoulder" in body_part_dict and "rightElbow" in body_part_dict:
            if body_part_dict["rightShoulder"][1] > body_part_dict["rightElbow"][1]:
                return True
        
        if "leftShoulder" in body_part_dict and "leftElbow" in body_part_dict:
            if body_part_dict["leftShoulder"][1] > body_part_dict["leftElbow"][1]:
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
        real_person_y = self.sensor_y - person_position[0]
        """
        distance = math.sqrt((real_person_x - self.sensor_x) ** 2 + (real_person_y - self.sensor_y) ** 2)
        # 三角形の相似を利用して算出
        safety_person_x = -(margin * (self.sensor_x - real_person_x)) / distance
        safety_person_y = -(margin * (self.sensor_y - real_person_y)) / distance
        """
        safety_person_x = real_person_x - margin
        safety_person_y = real_person_y
        
        return safety_person_x, safety_person_y
    
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
        if self.flag:
            return
        
        person_position = {}
        for pose in msgs.poses:
            
            if not self.is_raise_hand(pose.keypoints):
                continue
            
            result = self.calc_average_position(pose)
            if result == None:
                continue
            person_position.setdefault(result[0], result[1])
        
        # 手を上げている一番近い人間
        if len(person_position) == 0:
            del self.raise_hand_persons[:]
            return
        if min(person_position) < MARGIN:
            print "人間が近すぎます"
            del self.raise_hand_persons[:]
            return
        
        safety_person_position = self.calc_safe_position(MARGIN, person_position[min(person_position)])
        if len(self.raise_hand_persons) < 10:
            self.raise_hand_persons.append(safety_person_position)
            print "発見"
            self.se.play(self.se.DISCOVERY)
        else:
            status = self.send_move_base((self.raise_hand_persons[5][0], self.raise_hand_persons[5][1], 0))
            if status == actionlib.GoalStatus.SUCCEEDED:
                print "到着"
                self.flag = True
            else:
                del self.raise_hand_persons[:]
                print"失敗"


if __name__ == '__main__':
    RaiseHand()
    rospy.spin()
