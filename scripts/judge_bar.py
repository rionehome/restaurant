#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import rospy
from sensor_msgs.msg import LaserScan
from sound_system.srv import StringService, StringServiceRequest, StringServiceResponse


class JudgeBar:
    
    def __init__(self):
        rospy.init_node("rest_judge_bar")
        rospy.Service("/rest_judge_bar/detect", StringService, self.callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_sub)
        self.laser_data = None
    
    def callback(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        time.sleep(5)
        self.detect()
        return StringServiceResponse()
    
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
        # self.speak("My {} side is the nearest.".format("left"))
        else:
            direction = "left"
        # self.speak("My {} side is the nearest.".format("right"))
        
        self.speak("I am on the, {} side.".format(direction))
    
    def laser_sub(self, message):
        # type: (LaserScan) -> None
        self.laser_data = message
    
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


if __name__ == '__main__':
    JudgeBar()
    rospy.spin()
