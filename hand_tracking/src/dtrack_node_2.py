#!/usr/bin/env python3
import mediapipe as mp
import numpy as np
import cv2
from cvzone.HandTrackingModule import HandDetector
import rospy 
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class tracker_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_callback) 
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dep_image_callback)
        
        ### Publishers
        self.hand_position_pub = rospy.Publisher("hand_position", String, queue_size=1)
        self.image_position_pub = rospy.Publisher("image_position", Image, queue_size=1)
        
        ### Constants
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.bridge_object = CvBridge() # create the cv_bridge object
        self.detector = HandDetector(detectionCon=0.8, maxHands=2)
        
        ### Variables
        self.image_received = 0
        self.dep_received = 0
        self.centerPoint2 = None 
        self.centerPoint1 = None
        self.max_coord = 0
        
        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')

        while not rospy.is_shutdown():
            if self.image_received * self.dep_received == 0:
                print(self.dep_received)
                print(self.image_received)
                print('Image not received')
                continue
            self.hands, self.image = self.detector.findHands(self.image_cv, flipType=0, )  # with draw
            self.image_processing()
            self.publish()
            r.sleep()

    def risen_hand(self, hands_array):
        if self.hands_array[1] is None:
            self.max_coord = self.hands_array[0]
        else:
            self.max_coord = min((self.hands_array[0], self.hands_array[1]), key=lambda x: x[1])

    def image_processing(self):
        
        self.dep = self.depth_array

        if self.hands: #Obtain dictionary with all data given per hand
            # Hand 1
            hand1 = self.hands[0]
            self.centerPoint1 = hand1['center']  # center of the hand cx,cy  
            
            if len(self.hands) == 2:
                # Hand 2
                hand2 = self.hands[1]
                self.centerPoint2 = hand2['center']  # center of the hand cx,
            self.hands_array = (self.centerPoint1, self.centerPoint2)
            self.max_coord = self.risen_hand(self.hands_array)

        # Publish image    
        self.image = cv2.circle(self.cv_image, (self.coordx,self.coordy), 10, (0,0,255), 3)
        self.image_message = self.bridge_object.cv2_to_imgmsg(self.image, encoding="passthrough")

    def publish(self):
       
        # Image publisher
        self.image_position_pub.publish(self.image_message)

        os.system('clear') 
        print('Max coord: ',self.max_coord)
        
        # Depth hand calculation
        if self.max_coord[0] * self.max_coord[1] > 0:
            x = self.max_coord[0] * self.depx / self.image_width
            y = self.max_coord[1] * self.depy / self.image_height

            self.hand_depth = self.dep[int(y),int(x)] / 10
            print(self.dep.shape)
            print('The hand is at {} cm'.format(self.hand_depth))

    def camera_callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e) 
        if self.image_received == 0:
            self.image_height, self.image_width, c = self.cv_image.shape 
            self.image_received = 1

    def dep_image_callback(self,dep_image):
        self.depth_array = np.frombuffer(dep_image.data, dtype=np.uint16).reshape((dep_image.height, dep_image.width))
        if self.dep_received == 0:
            self.depy, self.depx = dep_image.height, dep_image.width
            self.dep_received = 1

    def cleanup(self):
               
        print('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('tracker_node', anonymous=True)
    tracker_node() 