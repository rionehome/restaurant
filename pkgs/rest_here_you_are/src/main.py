#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Restaurant Here you are

import rospy
from sound_system.srv import NLPService
from std_msgs.msg import String, Bool
from rest_start_node.msg import Activate
import time
import datetime
import os
import subprocess
from pocketsphinx import LiveSpeech


class Take:
	# ログファイルの書き込みの関数
	def log_file(self, sentence, judge):
		with open(self.log_file_name, "a") as f:
			if judge == "h":
				f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")
			elif judge == "s":
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

	def send_place_msg(self, place):
		# navigationに場所を伝える
		rospy.wait_for_service('/sound_system/nlp', timeout=1)
		response = rospy.ServiceProxy('/sound_system/nlp', NLPService)('Please go to {}'.format(place))
		print response.response
		if "OK" in response.response:
			self.navigation_wait = True
			while self.navigation_wait:
				time.sleep(0.1)
		else:
			# 次のノードに処理を渡す
			next = Activate()
			next.id = 2
			self.pub.publish(next)

	# 発話
	def speak(self, text):
		beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
		# PATH_beep_start = os.path.join(beep_path, 'start.wav')
		# PATH_beep_stop = os.path.join(beep_path, 'stop.wav')
		speech_wave = os.path.join(beep_path, 'speech.wav')
		# subprocess.call('aplay -q --quiet {}'.format(PATH_beep_stop), shell=True)
		subprocess.call(['pico2wave', '-w={}'.format(speech_wave), text])
		subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)

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
				print text
				return text
			else:
				print("**noise**")

	def navigation_callback(self, data):
		if self.speech_recognition:
			self.navigation_wait = False
			act = Activate()
			act.id = 3
			self.pub.publish(act)  # 商品を渡し終えたメッセージを送信

	# メッセージを受け取ったら、「Here you are」の発話
	def reach_customer(self, data):
		if data.id == 2:
			sentence = "Thank you for waiting. Here you are. Sorry, I have no arm. So, I want you to take items."
			self.log_file(sentence, "s")
			rospy.loginfo("robot spoke: %s", sentence)
			self.speak(sentence)
			time.sleep(10)
			self.speech_recognition = True  # yes no を受け取る

	# speech_recognitionがTrueになるまで待機するcallback関数
	def judge(self):
		while 1:
			if self.speech_recognition == True:
				break

	def main(self):
		self.judge()
		while 1:
			speak_text = "Did you take items?"
			self.speak(speak_text)
			self.log_file(speak_text, "s")
			rospy.loginfo("robot spoke: %s", speak_text)
			self.yes_no_resume()
			text = self.recognition()
			self.pause()
			if text == "yes":
				self.log_file("yes", "h")
				speak_text = "Thank you."
				self.speak(speak_text)
				self.log_file(speak_text, "s")
				rospy.loginfo("robot spoke: %s", speak_text)
				self.send_place_msg("kitchen")
				break
			else:
				self.log_file("no", "h")
				speak_text = "OK."
				self.speak(speak_text)
				self.log_file(speak_text, "s")
				rospy.loginfo("robot spoke: %s", speak_text)
				time.sleep(5)

	def __init__(self):
		rospy.init_node('rest_here_you_are_main', anonymous=True)
		rospy.Subscriber('/restaurant/activate', Activate, self.reach_customer)
		rospy.Subscriber('/navigation/goal', Bool, self.navigation_callback)
		self.pub = rospy.Publisher('/restaurant/activate', Activate, queue_size=10)  # キッチンに戻る
		self.model_path = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model'  # 音響モデルのディレクトリの絶対パス
		self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"),
												   datetime.datetime.now())
		self.speech_recognition = False
		self.main()
		self.navigation_wait = False
		rospy.spin()


if __name__ == '__main__':
	Take()
