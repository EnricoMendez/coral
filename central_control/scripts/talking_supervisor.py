#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

class supervisor():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        ### suscribers
        rospy.Subscriber("/zimmer/status",String,self.zimmer)
        rospy.Subscriber("/zimmer/inventory",Int32MultiArray,self.inventory)
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
        self.invetory_msg = []
        self.capacity = [0,1,1,2,2,2,1,1]

        r = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            os.system('clear')
            self.print_msg()
            r.sleep()

    def message_sort(self,msg,list):
        list = [list[1],list[2],list[3],list[4],list[5],msg]
        return list
    
    def talk(self,msg):
        tts = gTTS(msg, lang='en-us')
        tts.save('msg.mp3')
        os.system('mpg123 msg.mp3 > /dev/null 2>&1')
    
    def zimmer(self,data):
        self.zimmer_msg = self.message_sort(data.data,self.zimmer_msg)
        answer = data.data
        self.talk(answer)

    def inventory(self,data):
        self.invetory_msg = data.data

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
        print('Inventory:')
        for i in range(1,len(self.invetory_msg)):
            print('\tPiece ' + str(i) + ': ' + str(self.invetory_msg[i]) + '/' + str(self.capacity[i]))
           
              
        
    def cleanup(self):
        print('Node killed successfully')
        
if __name__ == "__main__":
    rospy.init_node('Supervisor',anonymous=False)
    supervisor()