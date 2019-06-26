#!/usr/bin/env python
# -*- coding: utf-8 -*-
# レストラン終了の音声認識、発話

import rospy
from std_msgs.msg import String, Bool
from rest_start_node.msg import Activate
import os
import subprocess
from pocketsphinx import LiveSpeech
import datetime


class Rest_Finish:
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

	# 発話
	def speak(self, text):
		beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
		speech_wave = os.path.join(beep_path, 'speech.wav')
		subprocess.call(['pico2wave', '-w={}'.format(speech_wave), text])
		subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)

	# ログファイルの書き込みの関数
	def log_file(self, sentence, judge):
		with open(self.log_file_name, "a") as f:
			if judge == "h":
				f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")
			elif judge == "s":
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

	# speech_recognitionがTrueになるまで待機するcallback関数
	def judge(self):
		while 1:
			if self.speech_recognition == True:
				break

	# キッチンに戻ってきた(音声認識開始の)メッセージを受け取る
	def control(self, data):
		if data.id == 3:
			self.speech_recognition = True

	def main(self):
		self.judge()
		while 1:
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
				act = Activate()
				act.id = 106
				self.pub.publish(act)
				break
			else:
				speak_text = "OK."
				rospy.loginfo("robot spoke: %s", speak_text)
				self.speak(speak_text)
				self.log_file(speak_text, "s")

	def __init__(self):
		rospy.init_node('rest_nlp_finish_main', anonymous=True)
		self.model_path = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model'  # 音響モデルのディレクトリの絶対パス
		self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"),
												   datetime.datetime.now())
		self.pub = rospy.Publisher('/restaurant/activate', Activate, queue_size=10)  # Restaurant終了
		rospy.Subscriber('/restaurant/activate', Activate, self.control)  # キッチンに戻ってきたらレストランをストップする音声認識開始
		self.speech = None
		self.speech_recognition = False
		print('== STOP RECOGNITION ==')
		self.main()
		rospy.spin()


if __name__ == '__main__':
	Rest_Finish()
