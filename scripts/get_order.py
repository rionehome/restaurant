#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from location.srv import RegisterLocation

from abstract_module import AbstractModule


class RestaurantGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantGetOrder, self).__init__(node_name="restaurant_get_order")
        
        rospy.Subscriber("/natural_language_processing/get_order", String, self.get_order_callback)
    
    def get_order_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingからのメッセージによって実行される
        お客さんを見つけることができたので、テーブルでオーダーを聞く
        :param: data: ""
        :return: なし
        """
        self.print_node("get_order")
        self.speak("OK.")
        # locationにtableの位置を記録
        # locationに車の位置を記録
        try:
            rospy.wait_for_service('/location/register_current_location', timeout=1)
            rospy.ServiceProxy('/location/register_current_location', RegisterLocation)("table")
            print "登録しました。"
        except rospy.ROSException:
            print "Error, not find location node."
        speak_sentence = "What is your order?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantGetOrder()
    rospy.spin()
