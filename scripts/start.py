#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from location.srv import RegisterLocation
from sound_system.srv import StringService
import time

from abstract_module import AbstractModule


class RestaurantStart(AbstractModule):
    def __init__(self):
        super(RestaurantStart, self).__init__(node_name="restaurant_start")

        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)

        rospy.Subscriber("/natural_language_processing/start", String, self.start_callback)

    def start_callback(self, data):
        # type: (String) -> None
        """
        1.テーブルの左右のジャッジ
        2.「"Hey Ducker"と呼んでください」と発話
        3.call_duckerを開始
        :param: data: なし
        :return: なし
        """
        self.print_node("start")

        time.sleep(3)

        rospy.wait_for_service("/rest_judge_bar/detect", timeout=1)
        direction = rospy.ServiceProxy("/rest_judge_bar/detect", StringService)()
        self.speak("I am on the {} side".format(direction))

        time.sleep(3)

        rospy.wait_for_service("/location/register_current_location", timeout=1)
        rospy.ServiceProxy("/location/register_current_location", RegisterLocation)("kitchen")

        self.speak("When ordering, please say, hey ducker.")
        # call_duckerにメッセージを送信
        self.call_ducker_pub.publish("start")


if __name__ == '__main__':
    RestaurantStart()
    rospy.spin()
