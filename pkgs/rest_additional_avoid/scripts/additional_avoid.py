#!/usr/bin/env python
# -*- coding: utf-8 -*-
from location.srv import RegisterLocation
import rospy
from sensor_msgs.msg import LaserScan
from sound_system.srv import *
from rest_start_node.msg import Activate
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import threading


class AdditionalAvoid:

    def __init__(self):
        rospy.init_node("rest_additional_avoid")
        rospy.Subscriber("/scan", LaserScan, self.laser_sub)
        rospy.Subscriber('/avoid/activate', Bool, self.activate_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_data = None
        self.activate = False
        self.obj_flag = False
        self.multi_thread()

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
        # type: (Bool) -> None
        self.activate = message.data

    def multi_thread(self):
        while not rospy.is_shutdown():
            if self.activate:
                if self.laser_data is not None:
                    if self.obj_flag:
                        self.cmd_pub.publish(Twist())
                    ranges = self.laser_data.ranges
                    length = 50
                    min_index = (len(ranges) // 2) - length
                    max_index = (len(ranges) // 2) + length
                    is_obj = False
                    for i in range(min_index, max_index):
                        r = ranges[i]
                        if r != 0 and r < 0.5:
                            is_obj = True
                            break
                    if is_obj:
                        if not self.obj_flag:
                            self.obj_flag = True
                            threading.Thread(target=self.speaking).start()
                    else:
                        if self.obj_flag:
                            self.obj_flag = False
                            threading.Thread(target=self.speaking2).start()

    def speaking(self):
        self.speak("I find Object")

    def speaking2(self):
        self.speak("I start navigation")

    def laser_sub(self, message):
        # type: (LaserScan) -> None
        self.laser_data = message


if __name__ == '__main__':
    AdditionalAvoid()
    rospy.spin()
