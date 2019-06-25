#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Restaurant 人に呼びかけられて角度を取得して発話

import rospy
from sound_system.srv import HotwordService
from std_msgs.msg import String, Bool
import time
import datetime
import os
import subprocess
import getting_array
from rest_start_node.msg import Activate

class Restaurant_find_sound:
	# 発話文のログファイル書き込みの関数
	def log_file_spoke(self, sentence):
		with open(self.log_file_name, "a") as f:
			f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

	def find_sound(self, data):
		# Hotword 検出処理
		rospy.wait_for_service("/hotword/detect", timeout=1)
		print "hotword待機"
		rospy.ServiceProxy("/hotword/detect", HotwordService)()

		angle = getting_array.read('DOAANGLE')
		subprocess.call('amixer sset Master 85% -q --quiet', shell=True)  # 大声
		sentence = "I will go"
		self.log_file_spoke(sentence)
		rospy.loginfo("robot spoke: %s", sentence)
		# ビープ音のディレクトリの絶対パスを変数に設定
		beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
		# PATH_beep_start = os.path.join(beep_path, 'start.wav')
		# PATH_beep_stop = os.path.join(beep_path, 'stop.wav')
		speech_wave = os.path.join(beep_path, 'speech.wav')
		# subprocess.call('aplay -q --quiet {}'.format(PATH_beep_stop), shell=True)
		subprocess.call(['pico2wave', '-w={}'.format(speech_wave), sentence])
		subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)
		subprocess.call('amixer sset Master 75% -q --quiet', shell=True)  # 声の大きさを戻す
		angle = str(angle)
		act = Activate()
		act.id = 100
		act.text = angle # 角度を送信
		self.pub.publish(act)
		print(angle)

	def __init__(self):
		rospy.init_node('rest_find_sound_start', anonymous=True)
		rospy.Subscriber('/rest_find_sound/find_sound', Activate, self.find_sound)
		self.pub = rospy.Publisher('/rest_find_sound/go_to_customer', Activate, queue_size=10) # お客さんの声の角度を送る
		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"), datetime.datetime.now())
		rospy.spin()

if __name__ == '__main__':
	Restaurant_find_sound()
