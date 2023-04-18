#!/usr/bin/env python3
import mediapipe as mp
import numpy as np
import cv2
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os
from tf2_geometry_msgs import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class tracker_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        
        ### Publishers
        self.hand_position_pub = rospy.Publisher("hand_position", String, queue_size=1)
        self.image_position_pub = rospy.Publisher("image_position", Image, queue_size=1)
        
        ### Constants
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.bridge_object = CvBridge() # create the cv_bridge object
        
        
        ### Variables
        self.image_received = 0
        self.coordinates_hands = np.ndarray([],[])
        
        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')

        while not rospy.is_shutdown():
            with self.mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as self.hands: 
                self.image_processing()
            r.sleep()
                
    def image_processing(self):
        os.system('clear')
        
        if self.image_received == 0:
            print('Image not received')
            return
        image = self.cv_image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.flip(image, 1)
        # Set flag
        image.flags.writeable = False
        # Detections
        results = self.hands.process(image)
        # Set flag to true
        image.flags.writeable = True
        # RGB 2 BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Get image shape

        
        # Rendering results
        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                self.mp_drawing.draw_landmarks(image, hand, self.mp_hands.HAND_CONNECTIONS, 
                                        self.mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                        self.mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                         )
                
            #
            
            c = 30
            i = 0
            for hand_landmarks in results.multi_hand_landmarks:
                self.coordinates_hands = np.append(hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * self.image_width, hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * self.image_height)
                text = (self.coordinates_hands[0], self.coordinates_hands[1])
                text = str(text)
                coordinates = (30,c)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.5
                color = (0,0,0)
                thickness = 1
                image = cv2.putText(image, text, coordinates, font, fontScale, color, thickness, cv2.LINE_AA)
                c = c+20
                i = i+1

                              

        # Publish image    
        image_message = self.bridge_object.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_position_pub.publish(image_message)
        
        if cv2.waitKey(10) & 0xFF == ord(' '):
            # print(results.multi_hand_landmarks)
            if results.multi_hand_landmarks is not None:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Position of landmark in middle finger MCP
                    
                    print( self.coordinates_hands[0], self.coordinates_hands[1])
                    #("Dimensions: ", image_width, image_height)
        elif cv2.waitKey(10) & 0xFF == ord('q'):
            return
        
        print(self.coordinates_hands)  
        self.coordinates_hands = np.empty

    def camera_callback(self,data):
        self.image_received = 1
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.image_height, self.image_width, c = self.cv_image.shape 
        except CvBridgeError as e:
            print(e) 

    def cleanup(self):
               
        print('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('tracker_node', anonymous=True)
    tracker_node() 