#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os

class supervisor():
    def __init__(self):
        ### suscribers
        rospy.Subscriber("/zimmer/status",String,self.zimmer)
        rospy.Subscriber("/move/status",String,self.move)
        rospy.Subscriber("/hand_track/status",String,self.hands)
        rospy.Subscriber("/object_class/status",String,self.object)
        rospy.Subscriber("/voice_recognition/status",String,self.voice)
        
        ### variables
        self.zimmer_msg = ['','','']
        self.move_msg = ['','','']
        self.hands_msg = ['','','']
        self.voice_msg = ['','','']
        self.object_msg = ['','','']

        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            os.system('clear')
            self.print_msg()
            r.sleep()

    def message_sort(self,msg,list):
        list = [msg,list[0],list[1]]
        return list
    
    def zimmer(self,data):
        self.zimmer_msg = self.message_sort(data.data,self.zimmer_msg)

    def move(self,data):
        self.move_msg = self.message_sort(data.data,self.move_msg)

    def hands(self,data):
        self.hands_msg = self.message_sort(data.data,self.hands_msg)

    def object(self,data):
        self.object_msg = self.message_sort(data.data,self.object_msg)
    
    def voice(self,data):
        self.voice_msg = self.message_sort(data.data,self.voice_msg)

    def print_msg(self):
        print('Zimmer node status:')
        for element in self.zimmer_msg:
            print('\t '+element)
        print('Voice recognition node status:')
        for element in self.voice_msg:
            print('\t'+element)
        print('Object classification node status:')
        for element in self.object_msg:
            print('\t'+element)
        print('Hands tracking node status:')
        for element in self.hands_msg:
            print('\t'+element)
        print('Robot control node status:')
        for element in self.move_msg:
            print('\t'+element)
        

if __name__ == "__main__":
    rospy.init_node('Supervisor',anonymous=False)
    supervisor()