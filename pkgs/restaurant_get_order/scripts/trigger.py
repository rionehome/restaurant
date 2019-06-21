#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
import os
import time

rospy.init_node('help', anonymous=True)
pub=rospy.Publisher('rest_ctrl', String, queue_size=10)
time.sleep(1)
pub.publish('')
