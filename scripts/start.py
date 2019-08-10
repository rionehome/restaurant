#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
from location.srv import RegisterLocation
from sound_system.srv import StringService

from abstract_module import AbstractModule


class RestaurantStart(AbstractModule):
    def __init__(self):
        super(RestaurantStart, self).__init__(node_name="restaurant_start")

        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "start":
            self.print_node(data.data)
            self.start()

    def start(self):
        # type: () -> None
        """
        1.テーブルの左右のジャッジ
        2.「"Hey Ducker"と呼んでください」と発話
        3.call_duckerを開始
        :return: なし
        """
        time.sleep(3)

        rospy.wait_for_service("/rest_judge_bar/detect")
        direction = rospy.ServiceProxy("/rest_judge_bar/detect", StringService)()
        self.speak("I am on the {} side".format(direction))

        time.sleep(3)

        rospy.wait_for_service("/navigation/register_current_location", timeout=1)
        rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("kitchen")

        self.speak("When ordering, please say, hey ducker.")
        # call_duckerにメッセージを送信
        self.call_ducker_pub.publish("start")


if __name__ == '__main__':
    RestaurantStart()
    rospy.spin()
