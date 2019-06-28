#!/usr/bin/env python
# -*- coding: utf-8 -*-
# レストラン終了の音声認識、発話

import rospy
from rest_start_node.msg import Activate
import os
from pocketsphinx import LiveSpeech
import datetime
from sound_system.srv import StringService


class RestFinish:
	def __init__(self, activate_id):
		rospy.init_node("rest_nlp_finish")
		rospy.Subscriber("/restaurant/activate", Activate, self.activation_callback)  # キッチンに戻ってきたらレストランをストップする音声認識開始

		self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"  # 音響モデルのディレクトリの絶対パス
		self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dictionary")  # 辞書のディレクトリの絶対パス
		self.pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)  # Restaurant終了

		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"),
												   datetime.datetime.now())

		self.speak_topic = "/sound_system/speak"
		self.id = activate_id
		self.speech = None
		self.speech_recognition = False
		print('== STOP RECOGNITION ==')
		self.main()

	def command_resume(self):
		print('== START RECOGNITION ==')
		self.speech = LiveSpeech(
			verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
			hmm=os.path.join(self.model_path, 'en-us'),
			lm=False,
			dic=os.path.join(self.dictionary_path, 'rest_finish_sphinx.dict'),
			jsgf=os.path.join(self.dictionary_path, 'rest_finish_sphinx.gram')
		)

	def yes_no_resume(self):
		print('== START RECOGNITION ==')
		self.speech = LiveSpeech(
			verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
			hmm=os.path.join(self.model_path, 'en-us'),
			lm=False,
			dic=os.path.join(self.dictionary_path, 'yes_no_sphinx.dict'),
			jsgf=os.path.join(self.dictionary_path, 'yes_no_sphinx.gram')
		)

	def pause(self):
		print('== STOP RECOGNITION ==')
		self.speech = LiveSpeech(no_search=True)

	# 音声認識結果の表示
	def recognition(self):
		for text in self.speech:
			score = text.confidence()
			if score > 0.1:
				text = str(text)
				print(text)
				return text
			else:
				print("**noise**")

	def speak(self, sentence):
		# type: (str) -> None
		"""
		発話関数
		:param sentence:
		:return:
		"""
		rospy.wait_for_service(self.speak_topic)
		rospy.ServiceProxy(self.speak_topic, StringService)(sentence)

	# ログファイルの書き込みの関数
	def log_file(self, sentence, judge):
		with open(self.log_file_name, "a") as f:
			if judge == "h":
				f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")
			elif judge == "s":
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

	# speech_recognitionがTrueになるまで待機するcallback関数
	def judge(self):
		while True:
			if self.speech_recognition:
				break

	# キッチンに戻ってきた(音声認識開始の)メッセージを受け取る
	def activation_callback(self, data):
		if data.id == self.id:
			self.speech_recognition = True

	def main(self):
		self.judge()
		while True:
			self.command_resume()  # 「wait」か「stop」を認識
			text = self.recognition()
			self.pause()  # 音声認識ストップ
			self.log_file(text, "h")
			speak_text = "May I stop the test? Please answer yes or no."
			rospy.loginfo("robot spoke: %s", speak_text)
			self.speak(speak_text)
			self.log_file(speak_text, "s")
			self.yes_no_resume()  # 「yes」か「no」を認識
			text = self.recognition()
			self.pause()  # 音声認識ストップ
			self.log_file(text, "h")
			if text == 'yes':
				speak_text = "OK. I will stop."
				rospy.loginfo("robot spoke: %s", speak_text)
				self.speak(speak_text)  # 終了
				self.log_file(speak_text, "s")
				activate = Activate()
				activate.id = self.id + 1
				self.pub.publish(activate)
				break
			else:
				speak_text = "OK."
				rospy.loginfo("robot spoke: %s", speak_text)
				self.speak(speak_text)
				self.log_file(speak_text, "s")


if __name__ == '__main__':
	RestFinish(3)
	rospy.spin()
