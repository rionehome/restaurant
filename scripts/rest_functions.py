#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Float64MultiArray
from sound_system.srv import *
from location.srv import RegisterLocation
import time


class RestaurantFunctions:
    def __init__(self):
        rospy.init_node("restaurant_rest_functions")

        self.place = "start_position"  # 場所名
        self.order = ""  # オーダーの商品名

        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        self.move_velocity_pub = rospy.Publisher("/move/velocity", Float64MultiArray, queue_size=10)
        self.function_name_pub = rospy.Publisher("/restaurant/function_argument_name", String, queue_size=10)

        rospy.Subscriber("/natural_language_processing/function_argument", String, self.function_argument_callback)
        rospy.Subscriber("/navigation/goal", Bool, self.navigation_goal_callback)

        self.start()

    def function_argument_callback(self, data):
        # type: (String) -> None
        """
        1.natural_language_processingから"関数名,引数名"を受け取る
        2.関数を実行する
        :param data:"関数名,引数名"
        :return: なし
        """
        function_argument_list = data.data.split(",", 1)
        # 引数が無いとき
        if function_argument_list[1] == "":
            eval("self." + function_argument_list[0])()
        else:
            eval("self." + function_argument_list[0])(function_argument_list[1])

    def start(self):
        # type: () -> None
        """
        1.テーブルの左右のジャッジ
        2.「"Hey Ducker"と呼んでください」と発話
        3.call_duckerを開始
        :return: なし
        """
        self.function_name_pub.publish("start")

    def call_ducker_finish_callback(self, msg):
        # type:(Bool) -> None
        """
        1.call_ducekrの成功/失敗を判定
        2.成功ならお客さんの確認、失敗ならcall_duckerをやり直す
        :return: なし
        """
        if msg.data:
            self.function_name_pub.publish("check_customer")
        else:
            self.function_name_pub.publish("restart_call_ducker")

    def restart_call_ducker(self):
        # type:() -> None
        """
        natural_language_processingからのメッセージによって実行される関数
        お客さんを間違っていたので、call_ducerのやり直し
        :return:なし
        """
        self.function_name_pub.publish("restart_call_ducker")

    def get_order(self):
        # type:() -> None
        """
        natural_language_processingからのメッセージによって実行される関数
        お客さんが合っていたので、テーブルでオーダーを聞く
        :return:なし
        """
        self.function_name_pub.publish("get_order")

    def restart_get_order(self, order):
        # type:(str) -> None
        """
        natural_language_processingからのメッセージによって実行される関数
        もう一度始めからオーダーを言い直してほしいと言う
        :param order: 商品名
        :return:なし
        """
        self.function_name_pub.publish("restart_get_order")

    def finish_get_order(self, order):
        # type:(str) -> None
        """
        natural_language_processingのメッセージによって実行される関数
        オーダーを記憶して、キッチンに移動する
        :return:なし
        """
        self.order = order
        self.place = "kitchen"
        self.function_name_pub.publish("finish_get_order")

    def kitchen(self):
        # type:() -> None
        """
        キッチンでオーダーを復唱
        :return:なし
        """
        self.speak("Order is {}.".format(self.order))
        self.speak("Please put order on the tray.")
        time.sleep(5)
        self.place = "table"
        self.send_place_msg("table")

    def here_you_are(self):
        # type:() -> None
        """
        テーブルで商品を渡す
        :return:なし
        """
        self.function_name_pub.publish("here_you_are")

    def restart_here_you_are(self):
        # type:() -> None
        """
        natural_language_processingのメッセージによって実行される関数
        少し待機してから、商品を取れたかどうかを、もう一度聞く
        :return:なし
        """
        self.function_name_pub.publish("restart_here_you_are")

    def finish_delivery(self):
        # type:() -> None
        """
        natural_language_processingのメッセージによって実行される関数
        商品の受け渡しが終了し、スタート位置（キッチン）に戻る
        :return:なし
        """
        self.place = "return start position"
        self.function_name_pub.publish("finish_delivery")

    def navigation_goal_callback(self, msg):
        # type:(Bool) -> None
        print "goal_callback"
        print msg.data
        if self.place == "start position":
            self.speak("sorry, I can not find you. Please call me again.")
            self.pub_move_velocity(0, 0)
            self.call_ducker_pub.publish("start")
        elif self.place == "kitchen":
            self.kitchen()
        elif self.place == "table":
            self.here_you_are()
        elif self.place == "return start position":
            self.speak("When ordering, please say, hey ducker.")
            self.pub_move_velocity(0, 0)
            self.place = "start position"
            self.call_ducker_pub.publish("start")

    def pub_move_velocity(self, straight, turn):
        """
        速度情報を送信
        :param straight:
        :param turn:
        :return:
        """
        array = Float64MultiArray()
        array.data.append(straight)
        array.data.append(0.03)
        array.data.append(turn)
        array.data.append(0.5)
        self.move_velocity_pub.publish(array)

    @staticmethod
    def send_place_msg(place):
        # type: (str) -> None
        """
        navigationに場所を伝える
        :param place: 場所
        :return:
        """
        rospy.wait_for_service("/sound_system/nlp", timeout=1)
        response = rospy.ServiceProxy("/sound_system/nlp", NLPService)('Please go to {}'.format(place))
        print response.response

    @staticmethod
    def speak(text):
        # type: (str) -> None
        """
        sound_system_ros に対して発話を要求する
        発話が終了するまで待機
        :param text: 発話内容
        :return: なし
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)


if __name__ == "__main__":
    RestaurantFunctions()
    rospy.spin()
