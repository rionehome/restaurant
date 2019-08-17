#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool

from abstract_module import AbstractModule


class RestaurantCheckCustomer(AbstractModule):
    def __init__(self):
        super(RestaurantCheckCustomer, self).__init__(node_name="restaurant_check_customer")
        
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        
        rospy.Subscriber("/call_ducker/finish", Bool, self.call_ducker_callback)
    
    def call_ducker_callback(self, msg):
        # type:(Bool) -> None
        """
        1.call_ducekrの成功/失敗を判定
        2.成功ならお客さんの確認、失敗ならcall_duckerをやり直す
        :param msg: Bool
        :return: なし
        """
        self.print_node("check_customer")
        if msg.data:
            self.print_node("check_customer")
            speak_sentence = "Are you ready to order?"
            self.speak(speak_sentence)
            self.nlp_pub.publish(speak_sentence)
        else:
            self.speak("Sorry, Please carry it in front of the customer.")
            print "お客さんの前に運んでください。"


if __name__ == '__main__':
    RestaurantCheckCustomer()
    rospy.spin()
