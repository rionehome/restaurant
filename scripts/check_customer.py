#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Float64MultiArray

from abstract_module import AbstractModule


class RestaurantCheckCustomer(AbstractModule):
    def __init__(self):
        super(RestaurantCheckCustomer, self).__init__(node_name="restaurant_check_customer")

        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        self.move_velocity_pub = rospy.Publisher("/move/velocity", Float64MultiArray, queue_size=10)

        rospy.Subscriber("/navigation_human_detect/goal", Bool, self.call_ducker_callback)

    def call_ducker_callback(self, msg):
        # type:(Bool) -> None
        """
        1.call_ducekrの成功/失敗を判定
        2.成功ならお客さんの確認、失敗ならcall_duckerをやり直す
        :return: なし
        """
        if msg.data:
            self.print_node("check_customer")
            speak_sentence = "Are you ready to order?"
            self.speak(speak_sentence)
            self.nlp_pub.publish(speak_sentence)

        else:
            self.speak("Sorry.")
            self.send_place_msg("kitchen")

            self.speak("sorry, I can not find you. Please call me again.")
            self.pub_move_velocity(0, 0)
            self.call_ducker_pub.publish("start")

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


if __name__ == '__main__':
    RestaurantCheckCustomer()
    rospy.spin()
