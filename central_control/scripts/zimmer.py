#!/usr/bin/env python3
import roslaunch
import rospy
import rospkg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import os


class zimmer():
    def __init__(self):
        ###******* INIT PUBLISHERS *******###  
        self.coord_pub=rospy.Publisher("coordinates_coral",Float32MultiArray,queue_size=10)
        ############################### SUBSCRIBERS #####################################   
        self.sub1 = rospy.Subscriber("/flag_coordinates",String,self.flag_callback) 
        self.sub2 = rospy.Subscriber("/move_finish",String,self.finish_callback) 
        ### Constants
        rospy.on_shutdown(self.cleanup)
        self.rospack = rospkg.RosPack()
        self.rospack.list()
        self.pkg_move = 'cobot_control'
        self.file_move = 'take_control.launch'
        self.pkg_bring = 'cobot_control'
        self.file_bring = 'urcom.launch'
        ### Variables
        self.coord_flag = False
        self.finish_flag = True
        self.bring = self.launch_file(self.pkg_bring,self.file_bring)

        
        ### Main loop ###
        print('Node initialized')
        self.bring.start()
        time.sleep(3)
        os.system('clear')
        print('Wait for communication')
        
        
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # print('Not finish')
            if self.finish_flag:
                self.finish_flag = False
                print('Im done')
                print('waiting time')
                # input()
                self.piano_ur()
            r.sleep()

    def flag_callback(self,data):
        self.coord_flag = True
        coordinates = Float32MultiArray()
        coordinates.data = [.5,.5,.5]
        self.coord_pub.publish(coordinates)
    
    def finish_callback(self,data):
        # os.system('clear')
        self.move.shutdown()
        time.sleep(3)
        self.bring.shutdown()
        print('Move has finished Ill wait')
        self.bring = self.launch_file(self.pkg_bring,self.file_bring)
        self.bring.start()
        time.sleep(1)
        self.finish_flag = True



    def piano_ur(self):                 
        os.system('clear')
        self.move = self.launch_file(self.pkg_move,self.file_move)
        self.move.start()
        print('wait for flag')
        while not self.coord_flag:
            pass
        print('Flag recieved')
        self.coord_flag = False

        

    def launch_file(self,pkg,file):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        hand_pkg_path = str(self.rospack.get_path(pkg))
        path_to_launch_file = hand_pkg_path+'/launch/'+file
        launch_file = roslaunch.rlutil.resolve_launch_arguments([path_to_launch_file])[0]
        output = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        return output

    def cleanup(self):
        print('Node killed')

if __name__ == "__main__":
    rospy.init_node('zimmer',anonymous=False)
    zimmer()