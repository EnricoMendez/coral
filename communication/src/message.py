#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Int32

class communication_ur_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Subscriber
        self.object_sub = rospy.Subscriber('object_num', Int32, self.object_num_callback)
        self.vr_commands_sub = rospy.Subscriber('vr_command', Int32, self.vr_command_callback)
        
        ### Publisher
        self.communication_ur_pub = rospy.Publisher("command", Int32, queue_size=1)
        
        ### Variables
        self.message = 0
        self.vr_command = 0
        self.object_number = 0

        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')
        while not rospy.is_shutdown():
            self.communication_ur(self.object_number, self.vr_command)
            self.publish()
            r.sleep()
    
    def message_arduino(self):
        self.message = 0
        #object_number = [0 1 2 3 4 5 6 7]
        #vr_command = [0 11 12 13 14 15 16 17 20 21 22 30 31]
        
        #Voice Recognition commands
        if self.vr_command == 31: #stop
            self.message = 31
            return self.message
        elif self.vr_command == 30: #cancel
            self.message = 30
            return self.message
        elif self.vr_command == 0: #no command
            self.message = self.message
        elif 11 <= self.vr_command <= 22:
            self.message = self.vr_command
        else: 
            self.message = 30
            
        #object classification
        if self.object_number == (8 or 0):  
            pass
        elif 0 < self.object_number < 8:   #no object
            self.message = self.object_number
        else: 
            self.message = 30
        return self.message

    def communication_ur(self,object_num, vr_command):
        self.message = self.message_arduino()
        
    def publish(self):
        self.communication_ur_pub.publish(self.message)
        os.system('clear') 
        print('Message: ', self.message)
        
    def object_num_callback(self, msg):
        # Callback function for object_num topic
        self.object_number = msg.data
        
    def vr_command_callback(self, msg):
        # Callback function for vr_command topic
        self.vr_command = msg.data

    def cleanup(self):    
        print('Node killed successfully')


if __name__ == '__main__':
    rospy.init_node('communication_ur_node', anonymous=True)
    communication_ur_node()
    