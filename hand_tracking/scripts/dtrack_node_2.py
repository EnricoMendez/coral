#!/usr/bin/env python3
import mediapipe as mp
import numpy as np
import cv2
from cvzone.HandTrackingModule import HandDetector
import rospy 
import os
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class tracker_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_callback) 
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dep_image_callback)
        
        ### Publishers
        self.hand_position_pub = rospy.Publisher("/hand_track/hand_position", Float32MultiArray, queue_size=1)
        self.status = rospy.Publisher("/hand_track/status", String, queue_size=1)

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
        self.max_coord = (None,None)
        self.hands_array = None
        self.hand_depth = None
        self.coord_x_hand1 = 0
        self.coord_x_hand2 = 0
        self.coord_y_hand1 = 0
        self.coord_y_hand2 = 0
        self.hand1 = None
        self.hand2 = None
        self.hand_position = None
        self.position_msg = Float32MultiArray()
        
        ###********** INIT NODE **********###  
        r = rospy.Rate(20)
        self.status.publish('initialized node')

        while not rospy.is_shutdown():
            if self.image_received * self.dep_received == 0:
                self.status.publish('Image not received')
                continue
            self.cv_image = cv2.flip(self.cv_image,1)
            self.hands, nimage = self.detector.findHands(self.cv_image, flipType=1, )  # with draw
            self.image_processing()
            self.publish()
            r.sleep()

    def risen_hand(self, hands_array):
        if hands_array[1] is None:
            return hands_array[0]
        else:
            return min((hands_array[0], hands_array[1]), key=lambda x: x[1])

    def image_processing(self):
        
        self.max_coord = None, None
        self.hands_array = None, None
        self.centerPoint1 = None
        self.centerPoint2 = None
        self.dep = self.depth_array

        if self.hands: #Obtain dictionary with all data given per hand
            # Hand 1

            self.hand1 = self.hands[0]
            self.coord_x_hand1 = self.hand1["lmList"][9][0]
            self.coord_y_hand1 = self.hand1["lmList"][9][1]
            self.centerPoint1 = (self.coord_x_hand1,self.coord_y_hand1)  # center of the hand cx,cy  
            
            if len(self.hands) == 2:
                # Hand 2
                self.hand2 = self.hands[1]
                self.coord_x_hand2 = self.hand2["lmList"][9][0]
                self.coord_y_hand2 = self.hand2["lmList"][9][1]
                self.centerPoint2 = (self.coord_x_hand2,self.coord_y_hand2) 

            self.hands_array = (self.centerPoint1, self.centerPoint2)
            self.max_coord = self.risen_hand(self.hands_array)
            for v in self.max_coord:
                if v < 0: 
                    v = 0
            if self.max_coord[0] > self.image_width:
                self.max_coord = self.image_width, self.max_coord[1]
            if self.max_coord[1] > self.image_height:
                self.max_coord = self.max_coord[0], self.image_height
        # Depth hand calculation
        if self.max_coord[0] is not None:
            x = int(self.max_coord[0] * self.depx / self.image_width)
            y = int(self.max_coord[1] * self.depy / self.image_height)
            if (0 < y < 480) and (0 < x < 848):
                if 100 >  (self.dep[y, x] / 10) > 0:
                    self.hand_depth = self.dep[y, x] / 10

        if self.hand_depth is not None and self.max_coord[0] is not None:
            # Conversion of values
            x_ratio = self.max_coord[0] / self.image_width
            y_ratio = 1 - (self.max_coord[1] / self.image_height)
            depth_ratio = self.hand_depth / 60

            # Limit values to range [0.0, 1.0]
            x_ratio = max(0.0, min(x_ratio, 1.0))
            y_ratio = max(0.0, min(y_ratio, 1.0))
            depth_ratio = max(0.0, min(depth_ratio, 1.0))

            # Round values to 3 decimal places
            x_ratio = round(x_ratio, 3)
            y_ratio = round(y_ratio, 3)
            depth_ratio = round(depth_ratio, 3)

            self.hand_position = x_ratio, y_ratio, depth_ratio


    def publish(self):

        self.position_msg.data = self.hand_position
        if self.hand_position is not None:
            msg = 'Hand position: '+ str(self.hand_position)
            self.status.publish(msg)
            self.hand_position_pub.publish(self.position_msg)

    def camera_callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        except CvBridgeError as e:
            self.status.publish(e) 
        if self.image_received == 0:
            self.image_height, self.image_width, c = self.cv_image.shape 
            self.image_received = 1

    def dep_image_callback(self,dep_image):
        self.depth_array = np.frombuffer(dep_image.data, dtype=np.uint16).reshape((dep_image.height, dep_image.width))
        if self.dep_received == 0:
            self.depy, self.depx = dep_image.height, dep_image.width
            self.dep_received = 1

    def cleanup(self):
               
        self.status.publish('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('tracker_node', anonymous=True)
    tracker_node() 