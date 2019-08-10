#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

from abstract_module import AbstractModule


class RestaurantRestartHereYouAre(AbstractModule):
    def __init__(self):
        super(RestaurantRestartHereYouAre, self).__init__(node_name="restaurant_restart_here_you_are")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "restart_here_you_are":
            self.restart_here_you_are()

    def restart_here_you_are(self):
        # type:() -> None
        """
        少し待機してから、商品を取れたかどうかを、もう一度聞く
        :return:なし
        """
        time.sleep(5)
        speak_sentence = "Did you take items?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantRestartHereYouAre()
    rospy.spin()
