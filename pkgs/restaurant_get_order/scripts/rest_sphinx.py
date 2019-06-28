#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import HotwordService, NLPService, StringService
from rest_start_node.msg import Activate
import get_order
import time
from collections import defaultdict


class RestGetOrder:
	def __init__(self, activate_id):
		rospy.init_node("restaurant_get_order")
		self.start_resume = rospy.Publisher("restaurant_getO/recognition_start", Bool, queue_size=10)
		self.yes_no = rospy.Publisher("yes_no/recognition_start", Bool, queue_size=10)
		# self.speak_pub = rospy.Publisher("/restaurant_nlp/speak", String, queue_size=10)  # 発話開始
		self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)

		rospy.Subscriber("/restaurant/activate", Activate, self.start_restaurant)  # 起動用
		rospy.Subscriber("yes_no/recognition_result", String, self.get_yesno)  # yes_no
		rospy.Subscriber("restaurant_getO/recognition_result", String, self.get_txt)  # 音声認識結果
		rospy.Subscriber("restaurant_nlp/finish_speaking", Bool, self.finish_speaking)  # 発話終了
		rospy.Subscriber("/navigation/goal", Bool, self.talk_order)

		self.id = activate_id
		self.txt = ""  # 音声認識の文字列を格納
		self.finish_speaking_flag = False
		self.take_answer = ""
		self.word_list = []  # 音声認識文からオーダを抽出
		self.menu_list = []  # word_listに商品個数を追加
		self.menu_dict = defaultdict(int)  # 商品個数計算用
		self.place = "start_position"
		self.activate_flag = False

	# 処理の開始
	def start_restaurant(self, data):
		# type: (Activate) -> None
		if data.id == 1:
			print "rest_get_order"
			self.activate_flag = True
			self.table()

	def send_place_msg(self, place):
		# type: (str) -> None
		# navigationに場所を伝える
		rospy.wait_for_service("/sound_system/nlp", timeout=1)
		response = rospy.ServiceProxy("/sound_system/nlp", NLPService)('Please go to {}'.format(place))
		print response.response
		if "OK" not in response.response:
			# 次のノードに処理を渡す
			activate = Activate()
			activate.id = self.id + 1
			self.activate_pub.publish(activate)

	@staticmethod
	def hot_word():
		rospy.wait_for_service("/hotword/detect", timeout=1)
		print "hot_word待機"
		rospy.ServiceProxy("/hotword/detect", HotwordService)()

	@staticmethod
	def speak(sentence):
		# type: (str) -> None
		rospy.wait_for_service("/sound_system/speak")
		rospy.ServiceProxy("/sound_system/speak", StringService)(sentence)

	def finish_speaking(self, data):
		if data.data:
			self.finish_speaking_flag = True

	# 音声認識結果の取得
	def get_txt(self, sentence):
		if sentence != "":
			self.txt = sentence.data
			return
		time.sleep(1)
		self.start_resume.publish(True)

	# yes/noの音声認識結果の取得
	def get_yesno(self, sentence):
		if not sentence == "":
			self.take_answer = sentence.data
			return
		print('I\'m taking yes or no...')
		time.sleep(1)
		self.yes_no.publish(True)

	# navigation終了
	def talk_order(self, data):
		# type: (Bool) -> None
		print data
		if not self.activate_flag:
			return

		if self.place == "table":
			activate = Activate()
			activate.id = self.id + 1
			self.activate_pub.publish(activate)
			self.activate_flag = False

		if self.place == "kitchen":
			self.kitchen()

	# 各商品が幾つずつあるかを計算
	def count_order(self, order_list):
		for i in [[key, 1] for key in order_list if key]:
			key, value = i
			self.menu_dict[key] += int(value)  # 各商品の個数をカウント
		for i in self.menu_dict:
			self.menu_list.append('{}'.format(self.menu_dict[i]) + i)  # 発話用に商品名の頭に個数をプラス
		return self.menu_list

	def kitchen(self):
		while True:
			self.speak("Order of Table A is")
			# オーダーを列挙していく
			for i in self.word_list:
				self.speak(i)

			self.speak("Is it OK?")

			self.take_answer = ""
			self.get_yesno("")
			while not self.take_answer == 'yes' and not self.take_answer == 'no':
				continue
			self.yes_no.publish(False)

			if self.take_answer == 'yes':
				# hot_word待機
				self.hot_word()
				self.speak("Please put order on the tray")

				while True:
					time.sleep(5)  # 商品が置かれるまで5秒待機
					self.speak("Did you put order on the tray?")
					self.get_yesno("")
					while not self.take_answer == 'yes' and not self.take_answer == 'no':
						continue
					self.yes_no.publish(False)
					if self.take_answer == 'yes':
						# 次への通信を書いてください
						# 制御へ場所情報を送信.
						self.place = "table"
						print "tableに移動するメッセージを投げる"
						self.send_place_msg(self.place)
						return
					else:
						continue
			else:
				self.speak("I say order again")

	def table(self):
		while True:
			self.speak("May i take your order?")
			self.txt = ""
			self.get_txt("")
			while self.txt == "":  # txt取得まで待機
				continue

			self.take_answer = ""
			for i in get_order.main(self.txt.decode('utf-8')):  # 注文されたメニューを取得
				self.word_list.append(i)

			self.speak("Anything else?")
			self.get_yesno("")  # 聴きとった内容が正しいかを確認
			while not self.take_answer == 'yes' and not self.take_answer == 'no':  # yesかnoを聞き取るまで待機
				continue
			if self.take_answer == 'yes':  # 注文がまだ終わってなければ再度注文を聞く
				continue

			self.word_list = self.count_order(self.word_list)  # 商品の個数をカウント

			self.speak("Let me confirm your order")

			for i in self.word_list:  # 確認のために商品を復唱
				self.speak(i)
			self.speak("Is it OK?")

			self.take_answer = ''
			self.get_yesno("")  # 聴きとった内容が正しいかを確認
			while not self.take_answer == 'yes' and not self.take_answer == 'no':  # yesかnoを聞き取るまで待機
				continue

			self.word_list = []  # 2週目以降に向けて初期化
			self.menu_list = []
			self.menu_dict = defaultdict(int)
			self.yes_no.publish(False)
			if self.take_answer == 'yes':
				self.speak("Sure")
				# 制御へ場所情報を送信.
				self.place = "kitchen"
				self.send_place_msg(self.place)
				return
			else:
				self.speak("Sorry, please say again your order from the beginning")


if __name__ == '__main__':
	RestGetOrder(1)
	rospy.spin()
