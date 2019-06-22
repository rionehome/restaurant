#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 発話 + ビープ音

import rospy
from std_msgs.msg import String, Bool
import subprocess
import os

class Speak:
    def callback(self, data):
        speech = data.data
        # ビープ音のディレクトリの絶対パス
        beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
        PATH_beep_start = os.path.join(beep_path, 'start.wav')
        PATH_beep_stop = os.path.join(beep_path, 'stop.wav')
        # ビープ音
        subprocess.call('aplay -q --quiet {}'.format(PATH_beep_start), shell=True)
        # 発話
        speech_wave =os.path.join(beep_path, 'speech.wav')
        subprocess.call(['pico2wave', '-w={}'.format(speech_wave), speech])
        subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)
        # ビープ音
        #subprocess.call('aplay -q --quiet {}'.format(PATH_beep_stop), shell=True)
        self.pub_finish.publish(True)

    def __init__(self):
        rospy.init_node('restaurant_getO_speak', anonymous=True)
        rospy.Subscriber('/restaurant_nlp/speak', String, self.callback) # 発話する文章
        self.pub_finish = rospy.Publisher('restaurant_nlp/finish_speaking', Bool, queue_size=10) # 発話終了の合図

if __name__ == '__main__':
    Speak()
    rospy.spin()

