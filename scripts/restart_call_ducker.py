#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantRestartCallDucker(AbstractModule):
    def __init__(self):
        super(RestaurantRestartCallDucker, self).__init__(node_name="restaurant_restart_call_ducker")

        rospy.Subscriber("/natural_language_processing/restart_call_ducker", String, self.restart_call_ducker_callback)

    def restart_call_ducker_callback(self, data):
        # type:(String) -> None
        """
        natural_language_processingからのメッセージによって実行される
        「Hey Ducker」の失敗　or お客さんを間違っていたので call_ducerのやり直し
        :return:なし
        """
        self.speak("Sorry.")
        self.send_place_msg("kitchen")


if __name__ == '__main__':
    RestaurantRestartCallDucker()
    rospy.spin()
