#!/usr/bin/env python3
import numpy as np
import rospy 
import os
from std_msgs.msg import String

class coordinates_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        self.positon_sub = rospy.Subscriber("/hand_position",String,self.position_callback) 
        
        ### Publisher
        self.hand_position_pub = rospy.Publisher("/hand_coordinates", String, queue_size=1)
        
        ### Constants
        self.pos_recieved = 0
        
        ### Variables
        self.cam_tuple = None
        self.coord_tuple = None, None, None
        self.conv_factor = 1

        
        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')

        while not rospy.is_shutdown():
            if self.cam_tuple is not None:
                self.calculate_coordinates()
                self.publish()
            r.sleep()

    def calculate_coordinates(self):
        self.conv_factor = self.cam_tuple[2] 
        self.coord_tuple = self.cam_tuple[0] * self.conv_factor,self.cam_tuple[1] * self.conv_factor , self.cam_tuple[2]



    def publish(self):

        #os.system('clear') 
        print(self.coord_tuple)
        self.hand_position_pub.publish(str(self.coord_tuple))


    def position_callback(self,data):
        if self.pos_recieved== 0:
            self.pos_received = 1
        pos_data = data.data
        self.cam_tuple = tuple(map(float, pos_data.strip("()").split(",")))

    def cleanup(self):
        print('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('coordinates_node', anonymous=True)
    coordinates_node() 