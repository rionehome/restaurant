#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sound_system.srv import *
from location.srv import RequestLocation
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Point, Quaternion
from module.rviz_marker import RvizMarker


class AbstractModule(object):
    def __init__(self, node_name):
        # type: (str) -> None
        
        rospy.init_node(node_name)
        self.nlp_pub = rospy.Publisher("/natural_language_processing/speak_sentence", String, queue_size=10)
        
        self.marker = RvizMarker()
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    
    def send_place_msg(self, place):
        # type: (str) -> None
        """
        navigationに場所を伝える
        :param place: 場所
        :return:
        """
        rospy.wait_for_service("/location/request_location", timeout=1)
        response = rospy.ServiceProxy("/location/request_location", RequestLocation)(place).location
        self.send_move_base(response)
    
    def send_move_base(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point.x, point.y, point.z)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.move_base_client.wait_for_server()
        print "move_baseに送信"
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_state()
    
    @staticmethod
    def speak(text):
        # type: (str) -> None
        """
        sound_system_ros に対して発話を要求する
        発話が終了するまで待機
        :param text: 発話内容
        :return: なし
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)
    
    @staticmethod
    def print_node(name):
        # type: (str) -> None
        """
        ノード名を表示
        :return: なし
        """
        print("\n###########################################\n")
        print("     Node: {}".format(name))
        print("\n###########################################\n")
