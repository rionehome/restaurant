#!/usr/bin/env python
# coding: UTF-8
import time

import rospy
from rest_start_node.msg import Activate

if __name__ == '__main__':
	rospy.init_node("rest_start_node")

	pub = rospy.Publisher('/restaurant/activate', Activate, queue_size=10)
	time.sleep(1)
	next = Activate()
	next.id = 0
	pub.publish(next)
