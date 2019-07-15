#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

from location.srv import RegisterLocation
import rospy
from sensor_msgs.msg import LaserScan
from sound_system.srv import *
from rest_start_node.msg import Activate


class JudgeBar:

	def __init__(self, activate_id):
		rospy.init_node("rest_judge_bar")
		rospy.Subscriber("/scan", LaserScan, self.laser_sub)
		rospy.Subscriber('/restaurant/activate', Activate, self.activate_callback)
		self.activate_pub = rospy.Publisher('/restaurant/activate', Activate, queue_size=10)
		self.laser_data = None
		self.id = activate_id

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

	def activate_callback(self, message):
		# type: (Activate) -> None
		if message.id == self.id:
			rospy.wait_for_service("/navigation/register_current_location", timeout=1)
			rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("kitchen")
			time.sleep(5)
			self.detect()
			self.activate_pub.publish(Activate(id=self.id + 1))

	def detect(self):
		while self.laser_data is None:
			pass
		ranges = self.laser_data.ranges
		right_index = (len(ranges) // 4) * 3
		left_index = (len(ranges) // 4) * 1
		left = 0
		right = 0
		length = 10

		count = 0
		for i in range(right_index - length, right_index + length):
			r = ranges[i]
			if r != 0:
				left += r
				count += 1
		if count == 0:
			left = 0.2
		else:
			left /= count

		count = 0
		for i in range(left_index - length, left_index + length):
			r = ranges[i]
			if r != 0:
				right += r
				count += 1
		if count == 0:
			right = 0.2
		else:
			right /= count

		self.speak("left is, {0:.2f} meters.".format(left))
		self.speak("right is, {0:.2f} meters.".format(right))
		print("left is, {0:.2f} meters.".format(left))
		print("right is, {0:.2f} meters.".format(right))

		if left < right:
			direction = "right"
			#self.speak("My {} side is the nearest.".format("left"))
		else:
			direction = "left"
			#self.speak("My {} side is the nearest.".format("right"))

		self.speak("I am on the, {} side, in Bar".format(direction))
		return

	def laser_sub(self, message):
		# type: (LaserScan) -> None
		self.laser_data = message


if __name__ == '__main__':
	JudgeBar(0)
	rospy.spin()
