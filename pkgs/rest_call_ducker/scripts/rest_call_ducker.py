#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from rest_start_node.msg import Activate


class RestCallDucker:
	def __init__(self, activate_id):
		rospy.init_node("rest_call_ducker")
		rospy.Subscriber("/call_ducker/finish", Bool, self.finish_callback)
		rospy.Subscriber("/restaurant/activate", Activate, self.activate_callback)
		self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
		self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
		self.id = activate_id

	def activate_callback(self, msg):
		# type: (Activate) -> None
		if msg.id == self.id:
			print "call_ducker"
			# call_duckerにメッセージを送信
			self.call_ducker_pub.publish("start")

	def finish_callback(self, msg):
		# type:(Bool) -> None
		if msg.data:
			# tableの場所を記録
			#
			##
			activate = Activate()
			activate.id = self.id + 1
			self.activate_pub.publish(activate)


if __name__ == '__main__':
	RestCallDucker(0)
	rospy.spin()
