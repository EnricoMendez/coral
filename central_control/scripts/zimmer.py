#!/usr/bin/env python3
import roslaunch
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import statistics
import os


class zimmer():
    def __init__(self):
        
        ###******* INIT PUBLISHERS *******###  
        self.coord_pub=rospy.Publisher("coordinates_coral",Float32MultiArray,queue_size=10)
        self.message_pub = rospy.Publisher('arduino_commands',Int32,queue_size=10)
        self.status_pub = rospy.Publisher('/zimmer/status',String,queue_size=10)
        self.inventory_pub = rospy.Publisher('/zimmer/inventory',Int32MultiArray,queue_size=10)

        ############################### SUBSCRIBERS #####################################   
        self.sub1 = rospy.Subscriber("/flag_coordinates",String,self.flag_callback) 
        self.sub2 = rospy.Subscriber("/move_finish",String,self.finish_callback) 
        self.sub3 = rospy.Subscriber("/voice_commands",String,self.vr_callback)
        self.sub4 = rospy.Subscriber("/hand_track/hand_position",Float32MultiArray,self.hand_track_callback)
        self.sub5 = rospy.Subscriber("/object_num",Int32,self.obj_num_callback)

        ### Constants
        rospy.on_shutdown(self.cleanup)
        self.rospack = rospkg.RosPack()
        self.rospack.list()
        self.pkg_move = 'cobot_control'
        self.file_move = 'take_control.launch'
        self.pkg_Ur_bring = 'cobot_control'
        self.file_Ur_bring = 'urcom.launch'

        ### Inventory

        self.inventory = [0,1,1,2,2,2,1,1]
        self.capacity = [0,1,1,2,2,2,1,1]

        ### Variables
        self.coord_flag = False
        self.finish_flag = False
        self.bring_flag = False
        self.take_flag = False
        self.inv_msg = Int32MultiArray()
        self.inv_msg.data = self.inventory
        self.inventory_pub.publish(self.inv_msg)
        self.Ur_bring = self.launch_file(self.pkg_Ur_bring,self.file_Ur_bring)
        self.coordinates = 0
        self.object_class = 0
        self.object_class_record = [0,0,0,0,0,0,0,0,0,0]
        self.index=0
        self.command=''
        self.com_flag = False
        self.mute = True
        ans = 'no'

        
        ### Main loop ###
        self.status_pub.publish('Zimmer initialized')
        self.Ur_bring.start()
        self.status_pub.publish('Comunication initialized')
        while ans != '':
            self.inventory_pub.publish(self.inv_msg)
            self.status_pub.publish('Press enter to kill communication')
            input('Click to kill communication')
            self.status_pub.publish('Communication killed')
            self.Ur_bring.shutdown()
            self.Ur_bring = self.launch_file(self.pkg_Ur_bring,self.file_Ur_bring)
            time.sleep(2)
            self.Ur_bring.start()
            self.com_flag = True
            self.status_pub.publish('Press enter when calibration proccess is complete')
            ans = input()
        self.inventory_pub.publish(self.inv_msg)
        self.mute = False
        r = rospy.Rate(10)
        self.status_pub.publish('Node initialized')
        while not rospy.is_shutdown():
            if self.com_flag:
                self.status_pub.publish('Waiting for take or bring command')
                while not (self.take_flag or self.bring_flag):
                    pass
                if self.take_flag:
                    self.status_pub.publish('Take routine')
                    self.drums_take()
                if self.bring_flag:
                    self.status_pub.publish('Bring routine')
                    self.guitar_bring()
            else: 
                pass
            r.sleep()

    def flag_callback(self,data):
        self.coord_flag = True
        self.coord_pub.publish(self.coordinates)

    def inventory_check(self,piece):
        if self.inventory[piece] > 0:
            self.inventory[piece] -= 1
            self.inv_msg.data = self.inventory
            self.inventory_pub.publish(self.inv_msg)
            return True
        else:
            msg = 'Piece '+str(piece)+' out of stock'
            self.status_pub.publish(msg)
            False
    
    def capacity_check(self,piece):
        if self.inventory[piece] < self.capacity[piece]:
            self.inventory[piece] += 1
            self.inv_msg.data = self.inventory
            self.inventory_pub.publish(self.inv_msg)
            return True
        else:
            msg = 'No more space available for piece ' + str(piece)
            self.status_pub.publish(msg)
            time.sleep(2)
            self.status_pub.publish('Say go to release the piece')
            time.sleep(1)
            while not self.command == 'go':
                pass
            self.message_pub.publish(22)
            self.take_flag = False
            return False
        
    def finish_callback(self,data):
        self.move.shutdown()
        self.com_flag = False
        self.Ur_bring.shutdown()
        self.finish_flag = True
        # self.status_pub.publish('Comunication killed')
        self.Ur_bring = self.launch_file(self.pkg_Ur_bring,self.file_Ur_bring)
        self.Ur_bring.start()
        # self.status_pub.publish('Comunication initialized')
        self.com_flag = True

    def guitar_bring(self):
        object = self.check_vr_num()
        while not object:
            if self.check_cancel():
                return
            object = self.check_vr_num()
        piece = object-10
        if not self.inventory_check(piece):
            return
        self.message_pub.publish(object)
        time.sleep(13)
        self.status_pub.publish('I will start external control')
        self.piano_ur()
        while not self.finish_flag:
            pass
        self.status_pub.publish('Waiting for go')
        while not self.command == 'go':
            pass
        self.message_pub.publish(22)
        self.bring_flag = False

    def check_vr_num(self):
        num_op = ['0','one','two','three','four','five','six','seven']
        

        if self.command in num_op:
            msg = str('I will go for piece ' + str(self.command))
            self.status_pub.publish(msg)
            return num_op.index(self.command) + 10        
        else:
            return 0

    def drums_take(self):
        self.status_pub.publish('I will start external control')
        # time.sleep(0.2)
        self.piano_ur()
        while not self.finish_flag:
            pass
        self.status_pub.publish('Waiting for go')
        while not self.command == 'go':
            if self.check_cancel():
                return
            pass
        self.message_pub.publish(22)
        time.sleep(5)
        tiempo_inicial = time.time()
        tiempo_actual = time.time()

        while tiempo_actual < tiempo_inicial + 6:
            if self.check_cancel():
                return
            tiempo_actual = time.time()
        if self.object_class == 8:
            self.status_pub.publish('No object detected')
            return
        self.message_pub.publish(self.object_class)
        if not self.capacity_check(self.object_class):
            return
        time.sleep(3)
        self.take_flag = False


    def check_cancel(self):
        if self.command == 'cancel':
                self.message_pub.publish(30)
                self.take_flag = False
                self.bring_flag = False
                self.status_pub.publish('Routine canceled')
                return True
        return False

    def clean(self):
        self.command = ''

    def piano_ur(self):
        self.finish_flag = False
        self.move = self.launch_file(self.pkg_move,self.file_move)
        self.move.start()
        # self.status_pub.publish('wait for flag')
        while not self.coord_flag:
            pass
        # self.status_pub.publish('Flag recieved')
        self.coord_flag = False

        

    def launch_file(self,pkg,file):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        hand_pkg_path = str(self.rospack.get_path(pkg))
        path_to_launch_file = hand_pkg_path+'/launch/'+file
        launch_file = roslaunch.rlutil.resolve_launch_arguments([path_to_launch_file])[0]
        output = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        return output
    


    def vr_callback(self,data):
        if self.mute:
            return
        self.command = data.data
        if self.take_flag or self.bring_flag :
            return
                                                                                                                                                                                                                                                                                                                                                                                                                                    
        if self.command == 'take':
            self.message_pub.publish(21)
            self.take_flag = True
        if self.command == 'bring':
            self.message_pub.publish(20)
            self.bring_flag = True

    def hand_track_callback(self,data):
        self.coordinates = data

    def obj_num_callback(self,data):
        self.object_class_record[int(self.index)] = int(data.data)
        if self.index == 9:
            self.index = 0
        else:
            self.index+=1
        self.object_class = statistics.mode(self.object_class_record)
        


        


    def cleanup(self):
        self.status_pub.publish('Node killed')

if __name__ == "__main__":
    rospy.init_node('zimmer',anonymous=False)
    zimmer()