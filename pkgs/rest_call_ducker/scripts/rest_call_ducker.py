#!/usr/bin/env python
# -*- coding: utf-8 -*-
from location.srv import RegisterLocation
import rospy
from sound_system.srv import StringService
from std_msgs.msg import String, Bool
from rest_start_node.msg import Activate


class RestCallDucker:
    def __init__(self, activate_id):
        rospy.init_node("rest_call_ducker")
        rospy.Subscriber("/navigation/goal", Bool, self.finish_callback)
        rospy.Subscriber("/restaurant/activate", Activate, self.activate_callback)
        self.call_ducker_pub = rospy.Publisher("/call_ducker/control", String, queue_size=10)
        self.activate_pub = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
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
    
    def activate_callback(self, msg):
        # type: (Activate) -> None
        if msg.id == self.id:
            print "call_ducker"
            self.speak("When ordering, please say, hey ducker.")  # 追加
            # call_duckerにメッセージを送信
            self.call_ducker_pub.publish("start")
    
    def finish_callback(self, msg):
        # type:(Bool) -> None
        # locationにtableの位置を記録
        rospy.wait_for_service("/navigation/register_current_location", timeout=1)
        rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("table")
        
        self.activate_pub.publish(Activate(id=self.id + 1))


if __name__ == '__main__':
    RestCallDucker(1)
    rospy.spin()
