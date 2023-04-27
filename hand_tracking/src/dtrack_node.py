#!/usr/bin/env python3
import mediapipe as mp
import numpy as np
import cv2
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
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = 0.5
        self.color = (0,0,0)
        self.thickness = 1
        
        ### Variables
        self.image_received = 0
        self.dep_received = 0
        self.coordinates_hands = np.zeros((2,1))
        self.max_coord = ''
        self.coordx = 0
        self.coordy = 0
        
        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')

        while not rospy.is_shutdown():
            if self.image_received * self.dep_received == 0:
                print(self.dep_received)
                print(self.image_received)
                print('Image not received')
                continue
            with self.mp_hands.Hands(min_detection_confidence=0.6, min_tracking_confidence=0.5) as self.hands: 
                self.image_processing()
                self.publish()
            r.sleep()

    def rising_hand(self,array1):
        max = np.argmax(array1[1])
        coord = array1[:,max]
        self.coordx = int(coord[0])
        self.coordy = self.image_height - int(coord[1]) 
        self.max_coord = str(self.coordx)+','+str(self.coordy)
        
    def image_processing(self):
        
        self.dep = self.depth_array
        image = self.cv_image
        # BGR 2 RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Flip on horizontal
        # image = cv2.flip(image, 1)
        # Set flag
        image.flags.writeable = False
        # Detections
        results = self.hands.process(image)
        # Set flag to true
        image.flags.writeable = True
        # RGB 2 BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        # Rendering results
        if results.multi_hand_landmarks:                         
            
            i = 0
            
            for hand_landmarks in results.multi_hand_landmarks:
                #Obtain coordinates of middle finger MCP
                x = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * self.image_width
                y = self.image_height - (hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * self.image_height)
                self.coordinates_hands = np.insert(self.coordinates_hands, i, [x,y], axis=1)               
                i = i+1        

            self.rising_hand(self.coordinates_hands)
            image = cv2.circle(image, (self.coordx,self.coordy), 10, (0,0,255), 3)

        # Publish image    
        self.image_message = self.bridge_object.cv2_to_imgmsg(image, encoding="passthrough")
        
        self.coordinates_hands = np.zeros((2,1))
    
    def publish(self):
       
        # Image publisher
        self.image_position_pub.publish(self.image_message)
        
        os.system('clear') 
        print('Max coord: ',self.max_coord)
        
        # Depth hand calculation
        if self.coordx * self.coordy > 0:
            x = self.coordx * self.depx / self.image_width
            y = self.coordy * self.depy / self.image_height

            self.hand_depth = self.dep[int(y),int(x)] / 10
            #print(self.dep.shape)
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