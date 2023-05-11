#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Int32

class SendGoalClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ###******* INIT PUBLISHERS *******### 
        self.command_pub = rospy.Publisher('command', Int32, queue_size=1)         
        
        ############################### SUBSCRIBERS #####################################
        rospy.Subscriber("input", Int32, self.execution)
        
        #********** INIT NODE **********### 
        r = rospy.Rate(20) #1Hz 

        #Do something else
        self.flag = 0
        while not rospy.is_shutdown(): 
            
            if self.flag == 1:
                self.command_pub.publish(self.comando_dato)
                self.flag = 0
            
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    def execution(self, msg):
        self.comando_dato = msg.data
        self.flag = 1
    
    def cleanup(self): 
        r.sleep()

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("ejemplo_coral", anonymous=True)
    SendGoalClass()                                       
