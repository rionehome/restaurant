#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

from abstract_module import AbstractModule


class RestaurantFinishGetOrder(AbstractModule):
    def __init__(self):
        super(RestaurantFinishGetOrder, self).__init__(node_name="restaurant_finish_get_order")

        rospy.Subscriber("/restaurant/function_name", String, self.function_name_callback)

    def function_name_callback(self, data):
        if data.data == "finish_get_order":
            self.print_node(data.data)
            self.finish_get_order()

    def finish_get_order(self):
        # type:() -> None
        """
        オーダーを記憶して、キッチンに移動する
        :return:なし
        """
        self.speak("Sure")
        # 制御へ場所情報を送信
        self.send_place_msg("kitchen")


if __name__ == '__main__':
    RestaurantFinishGetOrder()
    rospy.spin()
