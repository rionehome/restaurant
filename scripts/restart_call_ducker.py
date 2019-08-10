#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantRestartCallDucker(AbstractModule):
    def __init__(self):
        super(RestaurantRestartCallDucker, self).__init__(node_name="restaurant_restart_call_ducker")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "restart_call_ducker":
            self.print_node(data.data)
            self.restart_call_ducker()

    def restart_call_ducker(self):
        # type:() -> None
        """
        「Hey Ducker」の失敗　or お客さんを間違っていたので call_ducerのやり直し
        :return:なし
        """
        self.speak("Sorry.")
        self.send_place_msg("kitchen")


if __name__ == '__main__':
    RestaurantRestartCallDucker()
    rospy.spin()
