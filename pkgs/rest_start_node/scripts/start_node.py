#!/usr/bin/env python
# coding: UTF-8
import time

import rospy
from rest_start_node.msg import Activate

if __name__ == '__main__':
	rospy.init_node("hmc_start_node")

	pub = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)
	time.sleep(1)
	act = Activate
	act.id = 0
	pub.publish(act)
