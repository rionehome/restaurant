#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import HotwordService, NLPService, StringService
from rest_start_node.msg import Activate
import get_order
import time
from collections import defaultdict
from location.srv import RegisterLocation


class RestGetOrder:
    def __init__(self, activate_id):
        rospy.init_node("restaurant_get_order")
        self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
        self.change_dict_pub = rospy.Publisher("/sound_system/sphinx/dict", String, queue_size=10)
        self.change_gram_pub = rospy.Publisher("/sound_system/sphinx/gram", String, queue_size=10)
        
        rospy.Subscriber("/restaurant/activate", Activate, self.start_restaurant)  # 起動用
        rospy.Subscriber("/navigation/goal", Bool, self.talk_order)
        
        self.id = activate_id
        self.word_list = []  # 音声認識文からオーダを抽出
        self.menu_list = []  # word_listに商品個数を追加
        self.menu_dict = defaultdict(int)  # 商品個数計算用
        self.place = "start_position"
        self.activate_flag = False
    
    # 処理の開始
    def start_restaurant(self, data):
        # type: (Activate) -> None
        """
        前のノードからの信号受付
        :param data:
        :return:
        """
        if data.id == self.id:
            print "rest_get_order"
            rospy.wait_for_service("/navigation/register_current_location", timeout=1)
            rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("table")
            self.activate_flag = True
            self.table()
    
    def send_place_msg(self, place):
        # type: (str) -> None
        """
        navigationに場所を伝える
        :param place:
        :return:
        """
        rospy.wait_for_service("/sound_system/nlp", timeout=1)
        response = rospy.ServiceProxy("/sound_system/nlp", NLPService)('Please go to {}'.format(place))
        print response.response
        if "OK" not in response.response:
            # 次のノードに処理を渡す
            activate = Activate()
            activate.id = self.id + 1
            self.activate_pub.publish(activate)
    
    def resume_text(self, dict_name):
        # type: (str)->str
        """
        音声認識
        :return:
        """
        self.change_dict_pub.publish(dict_name + ".dict")
        self.change_gram_pub.publish(dict_name + ".gram")
        rospy.wait_for_service("/sound_system/recognition")
        response = rospy.ServiceProxy("/sound_system/recognition", StringService)()
        return response.response
    
    @staticmethod
    def hot_word():
        """
        「hey, ducker」に反応
        :return:
        """
        rospy.wait_for_service("/hotword/detect", timeout=1)
        print "hot_word待機"
        rospy.ServiceProxy("/hotword/detect", HotwordService)()
    
    @staticmethod
    def speak(sentence):
        # type: (str) -> None
        """
        speak関数
        :param sentence:
        :return:
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(sentence)
    
    # navigation終了
    def talk_order(self, data):
        # type: (Bool) -> None
        print data
        if not self.activate_flag:
            return
        
        if self.place == "table":
            self.activate_pub.publish(Activate(id=self.id + 1))
            self.activate_flag = False
        
        if self.place == "kitchen":
            self.kitchen()
    """
    def count_order(self, order_list):
        
        各商品が幾つずつあるかを計算
        :param order_list:
        :return:
        
        for i in [[key, 1] for key in order_list if key]:
            key, value = i
            self.menu_dict[key] += int(value)  # 各商品の個数をカウント
        for i in self.menu_dict:
            self.menu_list.append('{}'.format(self.menu_dict[i]) + i)  # 発話用に商品名の頭に個数をプラス
        return self.menu_list
    """
    
    def kitchen(self):
        """
        kitchenにいる時に実行する関数
        :return:
        """
        while True:
            self.speak("Order is")
            # オーダーを列挙していく
            for word in self.word_list:
                self.speak(word)
            
            self.speak("Is it OK?. Please answer yes or no.")
            
            # yes_no認識
            while True:
                take_answer = self.resume_text("yes_no_sphinx")
                if take_answer == "yes" or take_answer == "no":
                    break
            
            if take_answer == 'yes':
                # hot_word待機
                self.speak("when you finish cooking, Please call me, Hey Dukker.")
                self.speak("Please put order on the tray")
                
                while True:
                    time.sleep(5)  # 商品が置かれるまで5秒待機
                    self.speak("Did you put order on the tray?")
                    
                    # yes_no認識
                    while True:
                        take_answer = self.resume_text("yes_no_sphinx")
                        if take_answer == "yes" or take_answer == "no":
                            break
                    
                    if take_answer == 'yes':
                        # 次への通信を書いてください
                        # 制御へ場所情報を送信.
                        self.place = "table"
                        self.send_place_msg(self.place)
                        return
                    else:
                        continue
            else:
                self.speak("I say order again")
    
    def table(self):
        """
        tableにいる時に実行する関数
        :return:
        """
        self.word_list = []  # 2週目以降に向けて初期化
        self.menu_list = []
        self.menu_dict = defaultdict(int)

        while True:
            self.speak("What is your order?")
            while True:
                txt = self.resume_text("rest_menu_sphinx")
                if len(txt) > 2:
                    break
            
            for menu in get_order.main(txt.decode('utf-8')):  # 注文されたメニューを取得
                self.word_list.append(menu)
            #self.word_list = self.count_order(self.word_list)  # 商品の個数をカウント
            print(word_list)
            self.speak("Let me confirm your order.")
            rospy.sleep(1)
            for word in self.word_list:  # 確認のために商品を復唱
                self.speak(word)
            rospy.sleep(1)
            self.speak("Is {},OK?".format(self.word_list[0]))
            self.speak("Please answer, yes or no.")

            # yes_no認識
            while True:
                take_answer = self.resume_text("yes_no_sphinx")
                if take_answer == "yes" or take_answer == "no":
                    break
            
            if take_answer == 'yes':
                self.speak("Sure")
                # 制御へ場所情報を送信.
                self.place = "kitchen"
                self.send_place_msg(self.place)
                return
            else:
                self.speak("Sorry, please say again your order from the beginning")


if __name__ == '__main__':
    RestGetOrder(2)
    rospy.spin()
