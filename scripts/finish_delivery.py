#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantFinishDelivery(AbstractModule):
    def __init__(self):
        super(RestaurantFinishDelivery, self).__init__(node_name="restaurant_finish_delivery")
        
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        
        rospy.Subscriber("/natural_language_processing/finish_delivery", String, self.finish_delivery_callback)
    
    def finish_delivery_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingからのメッセージによって実行される
        商品の受け渡しを終了し、スタート位置（キッチン）に戻る
        :param data: ""
        :return: なし
        """
        self.print_node("finish_delivery")
        self.speak("Thank you.")
        self.send_place_msg("kitchen")
        
        # キッチンに着いた
        self.speak("When ordering, please say, hey ducker.")
        self.call_ducker_pub.publish("start")


if __name__ == '__main__':
    RestaurantFinishDelivery()
    rospy.spin()
