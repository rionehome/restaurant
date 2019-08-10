#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

from abstract_module import AbstractModule


class RestaurantFinishDelivery(AbstractModule):
    def __init__(self):
        super(RestaurantFinishDelivery, self).__init__(node_name="restaurant_finish_delivery")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "finish_delivery":
            self.print_node(data.data)
            self.finish_delivery()

    def finish_delivery(self):
        # type:() -> None
        """
        商品の受け渡しを終了し、スタート位置（キッチン）に戻る
        :return:なし
        """
        self.speak("Thank you.")
        self.send_place_msg("kitchen")


if __name__ == '__main__':
    RestaurantFinishDelivery()
    rospy.spin()