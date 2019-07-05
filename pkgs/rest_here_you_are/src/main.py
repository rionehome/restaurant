#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Restaurant Here you are

import rospy
from sound_system.srv import NLPService, StringService, HotwordService
from std_msgs.msg import Bool, String
from rest_start_node.msg import Activate
import time


class HereYouAre:
    
    def __init__(self, activate_id):
        rospy.init_node("rest_here_you_are")
        rospy.Subscriber("/restaurant/activate", Activate, self.reach_customer)
        rospy.Subscriber("/navigation/goal", Bool, self.navigation_callback)
        
        self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)  # キッチンに戻る
        self.change_dict_pub = rospy.Publisher("/sound_system/sphinx/dict", String, queue_size=10)
        self.change_gram_pub = rospy.Publisher("/sound_system/sphinx/gram", String, queue_size=10)
        self.id = activate_id
        self.activate_flag = False
    
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
    
    @staticmethod
    def hot_word():
        """
        「hey, ducker」に反応
        :return:
        """
        rospy.wait_for_service("/hotword/detect", timeout=1)
        print "hot_word待機"
        rospy.ServiceProxy("/hotword/detect", HotwordService)()
    
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
        if "OK" not in response.response:
            # 次のノードに処理を渡す
            self.activate_pub.publish(Activate(id=1))
            self.activate_flag = False
    
    def navigation_callback(self, data):
        if not self.activate_flag:
            return
        
        print data
        time.sleep(1)
        self.activate_pub.publish(Activate(id=1))  # hey ducker待機（始めに戻る）
    
    # メッセージを受け取ったら、「Here you are」の発話
    def reach_customer(self, data):
        if data.id == self.id:
            self.activate_flag = True
            self.speak("Thank you for waiting. Here you are. Sorry, I have no arm. So, I want you to take items.")
            time.sleep(10)
            self.main()
    
    def main(self):
        while True:
            self.speak("Did you take items?")
            
            # yes_no認識
            while True:
                text = self.resume_text("yes_no_sphinx")
                if text == "yes" or text == "no":
                    break
            
            if text == "yes":
                self.speak("Thank you.")
                self.send_place_msg("kitchen")
                break
            else:
                self.speak("OK.")
                time.sleep(5)


if __name__ == '__main__':
    HereYouAre(3)
    rospy.spin()
