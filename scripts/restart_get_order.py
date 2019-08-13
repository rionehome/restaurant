#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantRestartGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantRestartGetOrder, self).__init__(node_name="restaurant_restart_get_order")
        
        rospy.Subscriber("/natural_language_processing/restart_get_order", String, self.restart_get_order_callback)
    
    def restart_get_order_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingからのメッセージによって実行される関数
        もう一度始めからオーダーを言い直してほしいと言う
        :param data: 誤っている商品名
        :return: なし
        """
        self.print_node("restart_get_order")
        self.speak("Sorry, please say again your order from the beginning.")
        speak_sentence = "What is your order?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantRestartGetOrder()
    rospy.spin()
