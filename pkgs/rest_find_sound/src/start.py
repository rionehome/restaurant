#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Restaurant 人に呼びかけられて角度を取得して発話

import rospy
from std_msgs.msg import String, Bool
import time
import datetime
import difflib
import os
import subprocess
import getting_array

class Restaurant_find_sound:
	# 発話文のログファイル書き込みの関数
	def log_file_spoke(self, sentence):
		if os.path.exists(self.log_file_name) == True:
			with open(self.log_file_name, "a") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")
		else:
			with open(self.log_file_name, "w") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")
	# 声がしたメッセージを受け取ったら、角度を取得し、発話し、角度の情報を送る
	def find_sound(self, data):
		angle = getting_array.read('DOAANGLE')
		subprocess.call('amixer sset Master 85% -q --quiet', shell=True) # 大声
		sentence = "I will go"
		self.log_file_spoke(sentence)
		rospy.loginfo("robot spoke: %s", sentence)
		# ビープ音のディレクトリの絶対パスを変数に設定
		beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
		#PATH_beep_start = os.path.join(beep_path, 'start.wav')
		#PATH_beep_stop = os.path.join(beep_path, 'stop.wav')
		speech_wave =os.path.join(beep_path, 'speech.wav')
		#subprocess.call('aplay -q --quiet {}'.format(PATH_beep_stop), shell=True)
		subprocess.call(['pico2wave', '-w={}'.format(speech_wave), sentence])
		subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)
		subprocess.call('amixer sset Master 75% -q --quiet', shell=True) # 声の大きさを戻す
		self.pub.publish(str(angle))
		print(str(angle))


	def __init__(self):
		rospy.init_node('rest_find_sound_start', anonymous=True)
		rospy.Subscriber('rest_find_sound/find_sound', String, self.find_sound)
		self.pub = rospy.Publisher('rest_find_sound/go_to_customer', String, queue_size=10) # 制御に客まで行くメッセージを投げる
		self.log_file_flag = False # ログファイルを書き込みか追加か判定する
		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"), datetime.datetime.now())

if __name__ == '__main__':
	Restaurant_find_sound()
	rospy.spin()