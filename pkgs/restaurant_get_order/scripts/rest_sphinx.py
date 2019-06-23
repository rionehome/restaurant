#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from restaurant_get_order.msg import Order
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
            print(sentence)
            txt=sentence.data
            return
        time.sleep(1)
        start_resume.publish(True)

   
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
                start_speaking('May I take your order?')
                while (finish_speaking_flag != True):
		    continue

                txt=''
                get_txt('')
                while(txt==''):#txt取得まで待機
                    continue
                print(txt)
                take_ans=''
                word_list=[]
                word_list=get_order.main(txt.decode('utf-8'))
                
                start_speaking('Let me confirm your order')
                while (finish_speaking_flag != True):
		    continue

                for i in word_list:
                    #os.system("espeak '{}'".format(i))
                    start_speaking('{}'.format(i))
                    while (finish_speaking_flag != True):
		        continue
    
                start_speaking('Is it OK?')
                while (finish_speaking_flag != True):
		    continue
                
                get_yesno('')#聴きとった内容が正しいかを確認
                while(take_ans!='yes' and take_ans!='no'):#yesかnoを聞き取るまで待機
                    continue
                yes_no.publish(False)
                if(take_ans=='yes'):
                    #os.system("espeak 'Sure'")
                    start_speaking('Sure')
                    while (finish_speaking_flag != True):
		        continue
                    order.order=word_list
                    #send_order(Order)
                    start_flag=False
                    txt=''
                    break
                #制御へ場所情報を送信.
                else:
                    start_speaking('Sorry, please say again your order')
                    while (finish_speaking_flag != True):
		        continue
                    #os.system("espeak 'Sorry, please say again your order'")
                    txt=''
                    
                
            
            
    rospy.init_node('restaurant_getO', anonymous=True)
    start_resume=rospy.Publisher('restaurant_getO/resume/start', Bool, queue_size=10)
    yes_no=rospy.Publisher('yes_no/recognition_start', Bool, queue_size=10)
    speak=rospy.Publisher('/restaurant_nlp/speak', String, queue_size=10)#発話開始

    rospy.Subscriber('rest_ctrl', String, start_restaurant)#起動用
    rospy.Subscriber('yes_no/recognition_result', String, get_yesno)#yes_no
    rospy.Subscriber('restaurant_getO/resume/result', String, get_txt)#音声認識結果
    rospy.Subscriber('restaurant_nlp/finish_speaking', Bool, finish_speaking)#発話終了
    rospy.spin()

if __name__=='__main__':
    restaurant()
