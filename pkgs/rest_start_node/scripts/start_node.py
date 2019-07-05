#!/usr/bin/env python
# coding: UTF-8
import time
from sound_system.srv import NLPService, StringService, HotwordService
import rospy
from rest_start_node.msg import Activate
from std_msgs.msg import Bool, String

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
    rospy.init_node("rest_start_node")
    pub = rospy.Publisher('/restaurant/activate', Activate, queue_size=10)
    #---ここらか追加---
    #客にHey Dackerを呼ばせるために、レストラン開始時に発話(いきなり発話しないように3秒待機へ変更)
    time.sleep(3)
    speak('Please call me Hey Ducker')
    activate = Activate()
    activate.id = 0
    pub.publish(activate)
