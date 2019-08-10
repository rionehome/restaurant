#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantCheckCustomer(AbstractModule):
    def __init__(self):
        super(RestaurantCheckCustomer, self).__init__(node_name="restaurant_check_customer")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "check_customer":
            self.print_node(data.data)
            self.check_customer()

    def check_customer(self):
        # type:() -> None
        """
        呼んだお客さんなのかどうかを確認
        :return:なし
        """
        speak_sentence = "Are you ready to order?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantCheckCustomer()
    rospy.spin()
