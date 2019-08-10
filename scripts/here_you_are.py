#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

from abstract_module import AbstractModule


class RestaurantHereYouAre(AbstractModule):
    def __init__(self):
        super(RestaurantHereYouAre, self).__init__(node_name="restaurant_here_you_are")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "here_you_are":
            self.print_node(data.data)
            self.here_you_are()

    def here_you_are(self):
        # type:() -> None
        """
        テーブルで商品を渡す
        :return:なし
        """
        self.speak("Thank you for waiting. Here you are. So, I want you to take items.")
        time.sleep(5)
        speak_sentence = "Did you take items?"
        self.speak(speak_sentence)
        self.nlp_pub.publish(speak_sentence)


if __name__ == '__main__':
    RestaurantHereYouAre()
    rospy.spin()
