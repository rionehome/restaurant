#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from rest_start_node.msg import Activate


class Rest_Call_Ducker:
	def __init__(self):
		rospy.init_node("rest_call_ducker")
		rospy.Subscriber("/call_ducker/finish", Bool, self.finish_callback)
		rospy.Subscriber("/restaurant/activate", Activate, self.activate_callback)
		self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
		self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)

	def activate_callback(self, msg):
		# type: (Activate) -> None
		if msg.id == 0:
			print "call_ducker"
			# call_duckerにメッセージを送信
			self.call_ducker_pub.publish("start")

	def finish_callback(self, msg):
		# type:(Bool) -> None
		if msg.data:
			activate = Activate()
			activate.id = 1
			self.activate_pub.publish(activate)


if __name__ == '__main__':
	Rest_Call_Ducker()
	rospy.spin()
