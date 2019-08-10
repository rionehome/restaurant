#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from location.srv import RegisterLocation

from abstract_module import AbstractModule


class RestaurantGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantGetOrder, self).__init__(node_name="restaurant_get_order")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "get_order":
            self.print_node(data.data)
            self.get_order()

    def get_order(self):
        # type:() -> None
        """
        お客さんを見つけることができたので、テーブルでオーダーを聞く
        :return:なし
        """
        self.speak("OK.")
        # locationにtableの位置を記録
        rospy.wait_for_service("/navigation/register_current_location", timeout=1)
        rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("table")
        speak_sentence = "What is your order?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantGetOrder()
    rospy.spin()
