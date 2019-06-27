#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
import sys
from pocketsphinx import LiveSpeech
from hmc_start_node.msg import Activate


class Recognition:
	# 音声認識
	def resume(self):
		print('== START RECOGNITION ==')
		self.speech = LiveSpeech(
			verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
			hmm=os.path.join(self.model_path, 'en-us'),
			lm=False,
			dic=os.path.join(self.dictionary_path, 'menu_sphinx.dict'),
			jsgf=os.path.join(self.dictionary_path, 'menu_sphinx.gram')
		)

	# 音声認識ストップ
	def pause(self):
		print('== STOP RECOGNITION ==')
		self.speech = LiveSpeech(no_search=True)

	# 音声認識結果の表示
	def recognition(self):
		for text in self.speech:
			score = text.confidence()
			if score > 0.1:
				text = str(text)
				self.speech_recognition = False
				self.pub.publish(text)  # 音声認識の結果をpublish
				self.pause()
				break
			else:
				print("**noise**")

	# pauseかresumeの判定
	def judge(self):
		# stop_flag = False
		while 1:
			if self.speech_recognition == True:
				self.resume()
				self.recognition()
			# stop_flag = False
			# elif (self.speech_recognition == False) and (stop_flag == False):
			#	self.pause()
			#	stop_flag = True
			elif self.speech_recognition == "stop node":
				print("dubug")
				break

	# 音声認識再開のメッセージを受け取る
	def control(self, data):
		self.speech_recognition = data.data

	def control2(self, data):
		self.speech_recognition = data.data

	def control3(self, data):
		if data.id == 0:
			print "follow_me_nlp"
			self.speech_recognition = True

	def __init__(self):
		rospy.init_node('restaurant_getO_resume', anonymous=True)
		self.model_path = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model'  # 音響モデルのディレクトリの絶対パス
		self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
		rospy.Subscriber('restaurant_getO/recognition_start', Bool, self.control)
		# rospy.Subscriber('restaurant_getO/stop_recognition', String, self.control2)
		self.pub = rospy.Publisher('restaurant_getO/recognition_result', String, queue_size=10)
		self.speech = None
		self.speech_recognition = False
		print('== STOP RECOGNITION ==')
		self.judge()
		rospy.spin()


if __name__ == '__main__':
	Recognition()
