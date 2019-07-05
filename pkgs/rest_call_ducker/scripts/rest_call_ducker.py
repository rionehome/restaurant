#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sound_system.srv import NLPService
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
            self.speak("When ordering, please say ducker.") # 追加
            # call_duckerにメッセージを送信
            self.call_ducker_pub.publish("start")
    
    def finish_callback(self, msg):
        # type:(Bool) -> None
        if msg.data:
            # マイク切り替え
            
            # locationに車の位置を記録
            rospy.wait_for_service('/sound_system/nlp', timeout=1)
            print rospy.ServiceProxy('/sound_system/nlp', NLPService)('Here is table')
            
            self.activate_pub.publish(Activate(id=self.id + 1))


if __name__ == '__main__':
    RestCallDucker(1)
    rospy.spin()
