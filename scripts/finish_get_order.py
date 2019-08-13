#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

from abstract_module import AbstractModule


class RestaurantFinishGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantFinishGetOrder, self).__init__(node_name="restaurant_finish_get_order")
        
        rospy.Subscriber("/natural_language_processing/finish_get_order", String, self.finish_get_order_callback)
    
    def finish_get_order_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingのメッセージによって実行される関数
        オーダーを記憶して、キッチンに移動する
        :param data: 商品名
        :return: なし
        """
        self.print_node("finish_get_order")
        self.speak("Sure")
        self.send_place_msg("kitchen")
        
        # キッチンに着いた
        self.speak("Order is {}.".format(data.data))
        self.speak("Please give me items.")
        time.sleep(5)
        self.send_place_msg("table")
        
        # テーブルに着いた
        self.speak("Thank you for waiting. Here you are. So, I want you to take items.")
        time.sleep(5)
        speak_sentence = "Did you take items?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantFinishGetOrder()
    rospy.spin()
