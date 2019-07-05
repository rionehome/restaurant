#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import String
from sound_system.srv import *


class JudgeBar:

    def __init__(self):
        self.laser_data = None

        rospy.init_node("rest_judge_bar")
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_sub)
        self.detct_sub = rospy.Subscriber("/rest_judge_bar/detect", String, self.detect_callback)
        self.direction_pub = rospy.Publisher("/rest_judge_bar/direction", String, queue_size=10)
        print("起動")

    def detect_callback(self, message):
        # type: (String) -> None
        while self.laser_data is None:
            pass
        ranges = self.laser_data.ranges
        right_index = (len(ranges) // 4) * 3
        left_index = (len(ranges) // 4) * 1
        right = 0
        left = 0
        length = 10

        count = 0
        for i in range(right_index - length, right_index + length):
            r = ranges[i]
            if r != 0:
                right += r
                count += 1
        right /= count

        count = 0
        for i in range(left_index - length, left_index + length):
            r = ranges[i]
            if r != 0:
                left += r
                count += 1
        left /= count

        direction = None
        if right < left:
            direction = "right"
        else:
            direction = "left"
        self.direction_pub.publish("left")
        print("direction: left")
        rospy.wait_for_service("/sound_system/speak", timeout=1)
        rospy.ServiceProxy("/sound_system/speak", StringService)("I am on the {} side".format(direction))

    def laser_sub(self, message):
        # type: (LaserScan) -> None
        self.laser_data = message


if __name__ == '__main__':
    JudgeBar()
    rospy.spin()