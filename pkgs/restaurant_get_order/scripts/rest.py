#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from restaurant.msg import Order
import get_order
import os
import time
order=Order()

def restaurant():
    #処理の開始
    def start_restaurant(data):
        global start_flag
        start_flag=True
        main()
        return

    def speak_txt(txt):
        start_speaking(txt)
        while(finish_speaking_flag!=True):
            continue
        return

    
    def start_speaking(sentence):
        global finish_speaking_flag
        finish_speaking_flag=False
        if(sentence!=''):
            speak.publish(sentence)
            return

        
    def finish_speaking(data):
        global finish_speaking_flag
        if(data.data==True):
            finish_speaking_flag=True
            return


    #音声認識結果の取得
    def get_txt(sentence):
        global txt
        if(sentence!=''):
            txt=sentence.data
            return
        time.sleep(1)
        start_resume.publish('')

   
    #yes/noの音声認識結果の取得
    def get_yesno(sentence):
        global take_ans
        print(sentence)
        if(sentence!=''):
            take_ans=sentence.data
            print(take_ans)
            return
        print('I\'m taking yes or no...')
        time.sleep(1)
        yes_no.publish(True)

    def main():
        while(1):
            if(start_flag!=False):
                global take_ans, start_flag, loop_count, txt, finish_speaking_flag
                speak_txt('May I take your order?')

                txt=''
                get_txt('')
                while(txt==''):#txt取得まで待機
                    continue
                take_ans=''
                word_list=[]
                word_list=get_order.main(txt.decode('utf-8'))
                print(word_list)
                #os.system("espeak 'Let me confirm your order'")
                speak_txt('Let me confirm your order')

                for i in word_list:
                    #os.system("espeak '{}'".format(i))
                    speak_txt(i)
                speak_txt('Is it OK?')
                
                get_yesno('')#聴きとった内容が正しいかを確認
                while(take_ans!='yes' and take_ans!='no'):#yesかnoを聞き取るまで待機
                    continue
                yes_no.publish(False)
                if(take_ans=='yes'):
                    #os.system("espeak 'Sure'")
                    speak_txt('Sure')
                    order.order=word_list
                    #send_order(Order)
                    start_flag=False
                    txt=''
                    break
                #制御へ場所情報を送信.
                else:
                    speak_txt('Sorry, please say again your order')
                    #os.system("espeak 'Sorry, please say again your order'")
                    txt=''
                    
                
            
            
    rospy.init_node('restaurant_getO', anonymous=True)
    start_resume=rospy.Publisher('/sound_system/recognition', String, queue_size=10)
    yes_no=rospy.Publisher('yes_no/recognition_start', Bool, queue_size=10)
    speak=rospy.Publisher('/restaurant_nlp/speak', String, queue_size=10)#発話開始

    rospy.Subscriber('rest_ctrl', String, start_restaurant)#起動用
    rospy.Subscriber('yes_no/recognition_result', String, get_yesno)#yes_no
    rospy.Subscriber('sound_system/recognition/result', String, get_txt)#音声認識結果
    rospy.Subscriber('restaurant_nlp/finish_speaking', Bool, finish_speaking)#発話終了
    rospy.spin()

if __name__=='__main__':
    restaurant()
