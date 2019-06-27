#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import HotwordService, NLPService, StringService
from rest_start_node.msg import Activate
import get_order
import time


class RestGetOrder:
	def __init__(self, activate_id):
		rospy.init_node("restaurant_get_order")
		self.start_resume = rospy.Publisher("restaurant_getO/recognition_start", Bool, queue_size=10)
		self.yes_no = rospy.Publisher("yes_no/recognition_start", Bool, queue_size=10)
		self.speak = rospy.Publisher("/restaurant_nlp/speak", String, queue_size=10)  # 発話開始
		self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)

		rospy.Subscriber("/restaurant/activate", Activate, self.start_restaurant)  # 起動用
		rospy.Subscriber("yes_no/recognition_result", String, self.get_yesno)  # yes_no
		rospy.Subscriber("restaurant_getO/recognition_result", String, self.get_txt)  # 音声認識結果
		rospy.Subscriber("restaurant_nlp/finish_speaking", Bool, self.finish_speaking)  # 発話終了
		rospy.Subscriber("/navigation/goal", Bool, self.talk_order)

		self.id = activate_id
		self.txt = ""
		self.finish_speaking_flag = False
		self.take_answer = ""
		self.word_list = []
		self.place = "start_position"
		self.speak_topic = "/sound_system/speak"
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

	def start_speaking(self, sentence):
		# type: (str) -> None
		rospy.wait_for_service(self.speak_topic)
		rospy.ServiceProxy(self.speak_topic, StringService)(sentence)

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

	def kitchen(self):
		while True:
			self.start_speaking('Order of Table A is')
			# オーダーを列挙していく
			for i in self.word_list:
				self.start_speaking('{}'.format(i))
			self.start_speaking('Is it OK?')
			self.take_answer = ""
			self.get_yesno("")
			while not self.take_answer == 'yes' and not self.take_answer == 'no':
				continue
			self.yes_no.publish(False)
			if self.take_answer == 'yes':
				rospy.wait_for_service("/hotword/detect", timeout=1)
				print "hotword待機"
				rospy.ServiceProxy("/hotword/detect", HotwordService)()

				self.start_speaking('Please put order on the tray')
				while True:
					time.sleep(5)  # 商品が置かれるまで5秒待機
					self.start_speaking('Did you put order on the tray?')
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
				self.start_speaking('I say order again')

	def table(self):
		self.start_speaking('May I take your order?')
		while True:
			self.txt = ""
			self.get_txt("")
			while self.txt == "":  # txt取得まで待機
				continue

			self.take_answer = ""
			self.word_list = get_order.main(self.txt.decode('utf-8'))

			self.start_speaking('Let me confirm your order')

			for i in self.word_list:
				self.start_speaking('{}'.format(i))

			self.start_speaking('Is it OK?')

			self.get_yesno("")  # 聴きとった内容が正しいかを確認
			while not self.take_answer == 'yes' and not self.take_answer == 'no':  # yesかnoを聞き取るまで待機
				continue

			self.yes_no.publish(False)
			if self.take_answer == 'yes':
				self.start_speaking("Sure")
				# 制御へ場所情報を送信.
				self.place = "kitchen"
				self.send_place_msg(self.place)
				return
			else:
				self.start_speaking('Sorry, please say again your order')


if __name__ == '__main__':
	RestGetOrder(1)
	rospy.spin()
