#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import HotwordService, NLPService
from restaurant_get_order.msg import Order
from rest_start_node.msg import Activate
import get_order
import time

order = Order()
start_flag = False
PLACE = "start_position"
navigation_wait = False


class RestGetOrder:
	def __init__(self):
		rospy.init_node('restaurant_get_order')
		self.start_resume = rospy.Publisher('restaurant_getO/recognition_start', Bool, queue_size=10)
		self.yes_no = rospy.Publisher('yes_no/recognition_start', Bool, queue_size=10)
		self.speak = rospy.Publisher('/restaurant_nlp/speak', String, queue_size=10)  # 発話開始
		self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)

		rospy.Subscriber('/restaurant/activate', Activate, self.start_restaurant)  # 起動用
		rospy.Subscriber('yes_no/recognition_result', String, self.get_yesno)  # yes_no
		rospy.Subscriber('restaurant_getO/recognition_result', String, self.get_txt)  # 音声認識結果
		rospy.Subscriber('restaurant_nlp/finish_speaking', Bool, self.finish_speaking)  # 発話終了
		rospy.Subscriber('/navigation/goal', Bool, self.talk_order)

	# 処理の開始
	def start_restaurant(self, data):
		if data.id == 1:
			global start_flag
			print "rest_get_order"
			start_flag = True
			self.main()

	def send_place_msg(self, place):
		global navigation_wait
		# navigationに場所を伝える
		rospy.wait_for_service('/sound_system/nlp', timeout=1)
		response = rospy.ServiceProxy('/sound_system/nlp', NLPService)('Please go to {}'.format(place))
		print response.response
		if "OK" in response.response:
			navigation_wait = True
		else:
			# 次のノードに処理を渡す
			activate = Activate()
			activate.id = 2
			self.activate_pub.publish(activate)

	def start_speaking(self, sentence):
		global finish_speaking_flag
		finish_speaking_flag = False
		if sentence != '':
			self.speak.publish(sentence)
			return

	def finish_speaking(self, data):
		global finish_speaking_flag
		if data.data:
			finish_speaking_flag = True
			return

	# 音声認識結果の取得
	def get_txt(self, sentence):
		global txt
		if sentence != '':
			txt = sentence.data
			return
		time.sleep(1)
		self.start_resume.publish(True)

	# yes/noの音声認識結果の取得
	def get_yesno(self, sentence):
		global take_ans
		if sentence != '':
			take_ans = sentence.data
			return
		print('I\'m taking yes or no...')
		time.sleep(1)
		self.yes_no.publish(True)

	# キッチン到着後のオーダー復唱
	def talk_order(self, data):
		global take_ans, start_flag, PLACE, navigation_wait
		if not start_flag:
			navigation_wait = False
			print PLACE
			if PLACE == "table":
				activate = Activate()
				activate.id = 2
				self.activate_pub.publish(activate)
				start_flag = False

			if PLACE == "kitchen":
				while True:
					self.start_speaking('Order of Table A is')
					while not finish_speaking_flag:
						continue
					# オーダーを列挙していく
					for i in word_list:
						self.start_speaking('{}'.format(i))
						while not finish_speaking_flag:
							continue
					self.start_speaking('Is it OK?')
					while not finish_speaking_flag:
						continue
					take_ans = ''
					self.get_yesno('')
					while not take_ans == 'yes' and not take_ans == 'no':
						continue
					self.yes_no.publish(False)
					if take_ans == 'yes':
						rospy.wait_for_service("/hotword/detect", timeout=1)
						print "hotword待機"
						rospy.ServiceProxy("/hotword/detect", HotwordService)()

						self.start_speaking('Please put order on the tray')
						while not finish_speaking_flag:
							continue
						while True:
							time.sleep(5)  # 商品が置かれるまで5秒待機
							self.start_speaking('Did you put order on the tray?')
							while not finish_speaking_flag:
								continue
							self.get_yesno('')
							while not take_ans == 'yes' and not take_ans == 'no':
								continue
							self.yes_no.publish(False)
							if take_ans == 'yes':
								# 次への通信を書いてください
								# 制御へ場所情報を送信.
								PLACE = "table"
								print "tableに移動するメッセージを投げる"
								self.send_place_msg(PLACE)
								return
							else:
								continue

					# break
					else:
						self.start_speaking('I say order again')
						while not finish_speaking_flag:
							continue

	def main(self):
		if start_flag:
			global take_ans, start_flag, txt, finish_speaking_flag, word_list, PLACE
			self.start_speaking('May I take your order?')
			while not finish_speaking_flag:
				continue
			while True:
				txt = ''
				self.get_txt('')
				while txt == '':  # txt取得まで待機
					continue

				take_ans = ''
				word_list = []
				word_list = get_order.main(txt.decode('utf-8'))

				self.start_speaking('Let me confirm your order')
				while not finish_speaking_flag:
					continue

				for i in word_list:
					# os.system("espeak '{}'".format(i))
					self.start_speaking('{}'.format(i))
					while not finish_speaking_flag:
						continue

				self.start_speaking('Is it OK?')
				while not finish_speaking_flag:
					continue

				self.get_yesno('')  # 聴きとった内容が正しいかを確認
				while not take_ans == 'yes' and not take_ans == 'no':  # yesかnoを聞き取るまで待機
					continue
				self.yes_no.publish(False)
				if take_ans == 'yes':
					# os.system("espeak 'Sure'")
					self.start_speaking('Sure')
					while not finish_speaking_flag:
						continue
					order.order = word_list
					# send_order(Order)
					# start_flag = False
					txt = ''
					# 制御へ場所情報を送信.
					PLACE = "kitchen"
					self.send_place_msg(PLACE)
					return
				else:
					self.start_speaking('Sorry, please say again your order')
					while not finish_speaking_flag:
						continue
					# os.system("espeak 'Sorry, please say again your order'")
					txt = ''


if __name__ == '__main__':
	RestGetOrder()
	rospy.spin()
