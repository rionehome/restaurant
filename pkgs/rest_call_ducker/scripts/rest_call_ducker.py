#!/usr/bin/env python
# -*- coding: utf-8 -*-
from location.srv import RegisterLocation
import rospy
from sound_system.srv import StringService, NLPService
from std_msgs.msg import String, Bool, Float64MultiArray
from rest_start_node.msg import Activate


class RestCallDucker:
    def __init__(self, activate_id):
        rospy.init_node("rest_call_ducker")
        rospy.Subscriber("/navigation_human_detect/goal", Bool, self.finish_callback)
        rospy.Subscriber("/restaurant/activate", Activate, self.activate_callback)
        rospy.Subscriber("/navigation/goal", Bool, self.navigation_goal_callback)
        self.change_dict_pub = rospy.Publisher("/sound_system/sphinx/dict", String, queue_size=10)
        self.change_gram_pub = rospy.Publisher("/sound_system/sphinx/gram", String, queue_size=10)
        self.move_velocity_pub = rospy.Publisher("/move/velocity", Float64MultiArray, queue_size=10)
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
        self.id = activate_id
        self.activate_flag = False
    
    @staticmethod
    def speak(sentence):
        # type: (str) -> None
        """
        speak関数
        :param sentence:
        :return:
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(sentence)
    
    def send_place_msg(self, place):
        # type: (str) -> None
        """
        navigationに場所を伝える
        :param place:
        :return:
        """
        rospy.wait_for_service("/sound_system/nlp", timeout=1)
        response = rospy.ServiceProxy("/sound_system/nlp", NLPService)('Please go to {}'.format(place))
        print response.response
    
    def resume_text(self, dict_name):
        # type: (str)->str
        """
        音声認識
        :return:
        """
        self.change_dict_pub.publish(dict_name + ".dict")
        self.change_gram_pub.publish(dict_name + ".gram")
        rospy.wait_for_service("/sound_system/recognition")
        response = rospy.ServiceProxy("/sound_system/recognition", StringService)()
        return response.response
    
    def pub_move_velocity(self, straight, turn):
        """
        速度情報を送信
        :param straight:
        :param turn:
        :return:
        """
        array = Float64MultiArray()
        array.data.append(straight)
        array.data.append(0.03)
        array.data.append(turn)
        array.data.append(0.5)
        self.move_velocity_pub.publish(array)
    
    def activate_callback(self, msg):
        # type: (Activate) -> None
        if msg.id == self.id:
            print "call_ducker"
            self.activate_flag = True
            self.speak("When ordering, please say, hey ducker.")  # 追加
            # call_duckerにメッセージを送信
            self.call_ducker_pub.publish("start")
    
    def navigation_goal_callback(self, msg):
        if self.activate_flag:
            self.speak("sorry, I can not find you. Please call me again.")
            self.pub_move_velocity(0, 0)
            self.call_ducker_pub.publish("start")
            self.activate_flag = False
    
    def finish_callback(self, msg):
        # type:(Bool) -> None
        if msg.data:
            self.speak("Are you ready to order?")
            # yes_no認識
            while True:
                take_answer = self.resume_text("yes_no_sphinx")
                if take_answer == "yes" or take_answer == "no":
                    break
            
            if take_answer == "yes":
                self.speak("OK.")
                # locationにtableの位置を記録
                rospy.wait_for_service("/navigation/register_current_location", timeout=1)
                rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("table")
                self.activate_pub.publish(Activate(id=self.id + 1))
                self.activate_flag = False
            else:
                self.speak("Sorry.")
                self.send_place_msg("kitchen")
        else:
            self.speak("sorry, I can not find you. Please call me again.")
            self.pub_move_velocity(0, 0)
            self.call_ducker_pub.publish("start")
            self.activate_flag = False


if __name__ == '__main__':
    RestCallDucker(1)
    rospy.spin()
