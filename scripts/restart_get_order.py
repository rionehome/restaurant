#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantRestartGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantRestartGetOrder, self).__init__(node_name="restaurant_restart_get_order")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "restart_get_order":
            self.print_node(data.data)
            self.restart_get_order()

    def restart_get_order(self):
        # type:() -> None
        """
        natural_language_processingからのメッセージによって実行される関数
        もう一度始めからオーダーを言い直してほしいと言う
        :param order: 商品名
        :return:なし
        """
        self.speak("Sorry, please say again your order from the beginning")
        speak_sentence = "What is your order?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantRestartGetOrder()
    rospy.spin()
