#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

from abstract_module import AbstractModule


class RestaurantRestartHereYouAre(AbstractModule):
    def __init__(self):
        super(RestaurantRestartHereYouAre, self).__init__(node_name="restaurant_restart_here_you_are")
        
        rospy.Subscriber("/natural_language_processing/restart_here_you_are", String,
                         self.restart_here_you_are_callback)
    
    def restart_here_you_are_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingからのメッセージによって実行される
        少し待機してから、商品を取れたかどうかを、もう一度聞く
        :param data: ""
        :return: なし
        """
        self.print_node("restart_here_you_are")
        self.speak("OK.")
        time.sleep(5)
        speak_sentence = "Did you take items?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantRestartHereYouAre()
    rospy.spin()
