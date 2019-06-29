#!/usr/bin/env python
# -*- coding: utf-8 -*-
# レストラン終了の音声認識、発話

import rospy
from rest_start_node.msg import Activate
from sound_system.srv import StringService, HotwordService
from std_msgs.msg import String


class RestFinish:
	def __init__(self, activate_id):
		rospy.init_node("rest_nlp_finish")
		rospy.Subscriber("/restaurant/activate", Activate, self.activation_callback)  # キッチンに戻ってきたらレストランをストップする音声認識開始

		self.pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)  # Restaurant終了
		self.change_dict_pub = rospy.Publisher("/sound_system/sphinx/dict", String, queue_size=10)
		self.change_gram_pub = rospy.Publisher("/sound_system/sphinx/gram", String, queue_size=10)
		self.id = activate_id

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

	# キッチンに戻ってきた(音声認識開始の)メッセージを受け取る
	def activation_callback(self, data):
		if data.id == self.id:
			self.main()

	def main(self):
		while True:

			while True:
				text = self.resume_text("rest_finish_sphinx")
				if text == "wait" or text == "stop" or text == "stop the test":
					break

			self.speak("May I stop the test? Please answer yes or no.")

			# yes_no認識
			while True:
				text = self.resume_text("yes_no_sphinx")
				if text == "yes" or text == "no":
					break

			if text == 'yes':
				self.speak("OK. I will stop.")  # 終了
				break
			else:
				self.speak("OK.")


if __name__ == '__main__':
	RestFinish(3)
	rospy.spin()
