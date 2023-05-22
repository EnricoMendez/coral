#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
from std_msgs.msg import Int32
import pyttsx3

class supervisor():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ### suscribers
        rospy.Subscriber("/zimmer/status",String,self.zimmer)
        rospy.Subscriber("/move/status",String,self.move)
        rospy.Subscriber("/hand_track/status",String,self.hands)
        rospy.Subscriber("/object_class/status",String,self.object)
        rospy.Subscriber("/voice_recognition/status",String,self.voice)
        rospy.Subscriber("/arduino_commands",Int32,self.arduino)
        
        
        ### variables
        self.zimmer_msg = ['','','','','','']
        self.move_msg = ['','','','','','']
        self.hands_msg = ['','','','','','']
        self.voice_msg = ['','','','','','']
        self.object_msg = ['','','','','','']
        self.arduino_msg = ['','','','','','']


        ### Set voice configuration
        self.text_speech = pyttsx3.init(driverName='espeak')
        voices = self.text_speech.getProperty('voices')
        self.text_speech.setProperty('voice', 'google')
        rate = self.text_speech.getProperty('rate')
        self.text_speech.setProperty('rate', 175)

        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            os.system('clear')
            self.print_msg()
            r.sleep()

    def message_sort(self,msg,list):
        list = [list[1],list[2],list[3],list[4],list[5],msg]
        return list
    
    def zimmer(self,data):
        self.zimmer_msg = self.message_sort(data.data,self.zimmer_msg)
        answer = data.data
        self.text_speech.say(answer)
        self.text_speech.runAndWait()

    def move(self,data):
        self.move_msg = self.message_sort(data.data,self.move_msg)

    def hands(self,data):
        self.hands_msg = self.message_sort(data.data,self.hands_msg)

    def object(self,data):
        self.object_msg = self.message_sort(data.data,self.object_msg)
    
    def voice(self,data):
        self.voice_msg = self.message_sort(data.data,self.voice_msg)

    def arduino(self,data):
        self.arduino_msg = self.message_sort(str(data.data),self.arduino_msg)


    def print_msg(self):
        print('Zimmer node status:')
        for element in self.zimmer_msg:
            print('\t '+element)
        print('Voice recognition node status:')
        for element in self.voice_msg:
            print('\t'+element)
        print('Object classification node status:')
        print('\t'+self.object_msg[5])
        # for element in self.object_msg:
        #     print('\t'+element)
        print('Hands tracking node status:')
        # for element in self.hands_msg:
        #     print('\t'+element)     
        print('\t'+self.hands_msg[5])   
        print('Arduino commands:')
        for element in self.arduino_msg:
            print('\t'+element)
        print('Robot control node status:')
        for element in self.move_msg:
            print('\t'+element)
        
    def cleanup(self):
        print('Node killed successfully')
        
if __name__ == "__main__":
    rospy.init_node('Supervisor',anonymous=False)
    supervisor()