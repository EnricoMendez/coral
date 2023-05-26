#!/usr/bin/env python3

# Import necessary libraries and modules
import mediapipe as mp  
import numpy as np  
import cv2  
import rospy  
import os 
from cvzone.HandTrackingModule import HandDetector 
from std_msgs.msg import String, Float32MultiArray  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge, CvBridgeError 

class tracker_node():
    def __init__(self):
        # Initialize ROS node and set up subscribers and publishers
        rospy.on_shutdown(self.cleanup)  # Set cleanup method to be called when node is shutting down
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)  
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.dep_image_callback) 
        self.hand_position_pub = rospy.Publisher("/hand_track/hand_position", Float32MultiArray, queue_size=1) 
        self.status = rospy.Publisher("/hand_track/status", String, queue_size=1)

        # Constants and variables
        self.mp_drawing = mp.solutions.drawing_utils  # Drawing utilities from Mediapipe
        self.mp_hands = mp.solutions.hands  # Hands module from Mediapipe
        self.bridge_object = CvBridge()  # OpenCV and ROS image conversion
        self.detector = HandDetector(detectionCon=0.8, maxHands=2)  # Hand detector object with parameters
        self.image_received = 0  # Flag to check if color image is received
        self.dep_received = 0  # Flag to check if depth image is received
        self.centerPoint2 = None  # Center point of the second hand
        self.centerPoint1 = None  # Center point of the first hand
        self.max_coord = (None, None)  # Maximum coordinates of the detected hand
        self.hands_array = None  # Array containing the center points of both hands
        self.hand_depth = None  # Depth value of the detected hand
        self.coord_x_hand1 = 0  # X-coordinate of the first hand
        self.coord_x_hand2 = 0  # X-coordinate of the second hand
        self.coord_y_hand1 = 0  # Y-coordinate of the first hand
        self.coord_y_hand2 = 0  # Y-coordinate of the second hand
        self.hand1 = None  # Data dictionary for the first hand
        self.hand2 = None  # Data dictionary for the second hand
        self.hand_position = None  # Hand position (x_ratio, y_ratio, depth_ratio)
        self.position_msg = Float32MultiArray()  # ROS message for hand position

        # Initialize the ROS node
        rospy.init_node('tracker_node', anonymous=True)
        r = rospy.Rate(20)  # Rate at which the node runs

        # Main loop of the node
        while not rospy.is_shutdown():
            if self.image_received * self.dep_received == 0:
                self.status.publish('Image not received')
                continue
            self.cv_image = cv2.flip(self.cv_image, 1)  # Flip the color image horizontally
            self.hands, nimage = self.detector.findHands(self.cv_image, flipType=1)  # Detect hands in the flipped image
            self.image_processing()  # Perform image processing
            self.publish()  # Publish the hand position
            r.sleep()  # Sleep to maintain the rate of the node

    def risen_hand(self, hands_array):
        # Determine the hand with the highest y-coordinate (closest to the top of the image)
        if hands_array[1] is None:
            return hands_array[0]
        else:
            return min((hands_array[0], hands_array[1]), key=lambda x: x[1])

    def image_processing(self):
        # Process the color and depth images to obtain hand position and depth

        self.max_coord = None, None
        self.hands_array = None, None
        self.centerPoint1 = None
        self.centerPoint2 = None
        self.dep = self.depth_array  # Depth array from the depth image

        if self.hands:  # If hands are detected
            self.hand1 = self.hands[0]  # Data dictionary for the first hand
            self.coord_x_hand1 = self.hand1["lmList"][9][0]  # X-coordinate of the tip of the index finger of the first hand
            self.coord_y_hand1 = self.hand1["lmList"][9][1]  # Y-coordinate of the tip of the index finger of the first hand
            self.centerPoint1 = (self.coord_x_hand1, self.coord_y_hand1)  # Center point of the first hand

            if len(self.hands) == 2:  # If two hands are detected
                self.hand2 = self.hands[1]  # Data dictionary for the second hand
                self.coord_x_hand2 = self.hand2["lmList"][9][0]  # X-coordinate of the tip of the index finger of the second hand
                self.coord_y_hand2 = self.hand2["lmList"][9][1]  # Y-coordinate of the tip of the index finger of the second hand
                self.centerPoint2 = (self.coord_x_hand2, self.coord_y_hand2)  # Center point of the second hand

            self.hands_array = (self.centerPoint1, self.centerPoint2)  # Array containing the center points of both hands
            self.max_coord = self.risen_hand(self.hands_array)  # Get the hand with the highest y-coordinate

            # Ensure that the coordinates are within the image bounds
            for v in self.max_coord:
                if v < 0:
                    v = 0
            if self.max_coord[0] > self.image_width:
                self.max_coord = self.image_width, self.max_coord[1]
            if self.max_coord[1] > self.image_height:
                self.max_coord = self.max_coord[0], self.image_height

        if self.max_coord[0] is not None:  # If a hand is detected
            x = int(self.max_coord[0] * self.depx / self.image_width)  # Calculate x-coordinate in depth image
            y = int(self.max_coord[1] * self.depy / self.image_height)  # Calculate y-coordinate in depth image
            if (0 < y < self.depy) and (0 < x < self.depx):  # Ensure coordinates are within depth image bounds
                if 100 > (self.dep[y, x] / 10) > 0:  # Convert depth value from mm to cm
                    self.hand_depth = self.dep[y, x] / 10

        if self.hand_depth is not None and self.max_coord[0] is not None:  # If hand depth and position are available
            # Conversion of values
            x_ratio = self.max_coord[0] / self.image_width  # X-coordinate ratio (0.0 to 1.0)
            y_ratio = 1 - (self.max_coord[1] / self.image_height)  # Y-coordinate ratio (0.0 to 1.0)
            depth_ratio = self.hand_depth / 60  # Depth ratio (0.0 to 1.0)

            # Limit values to range [0.0, 1.0]
            x_ratio = max(0.0, min(x_ratio, 1.0))
            y_ratio = max(0.0, min(y_ratio, 1.0))
            depth_ratio = max(0.0, min(depth_ratio, 1.0))

            # Round values to 3 decimal places
            x_ratio = round(x_ratio, 3)
            y_ratio = round(y_ratio, 3)
            depth_ratio = round(depth_ratio, 3)

            self.hand_position = x_ratio, y_ratio, depth_ratio  # Hand position as (x_ratio, y_ratio, depth_ratio)

    def publish(self):
        # Publish the hand position as a ROS message

        self.position_msg.data = self.hand_position  # Set the position data in the ROS message
        if self.hand_position is not None:  # If hand position is available
            msg = 'Hand position: ' + str(self.hand_position)  # Create status message
            self.status.publish(msg)  # Publish the status message
            self.hand_position_pub.publish(self.position_msg)  # Publish the hand position message

    def camera_callback(self, data):
        # Callback function for the color image subscriber

        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")  # Convert ROS image to OpenCV image
        except CvBridgeError as e:
            self.status.publish(e)

        if self.image_received == 0:  # If color image is not received before
            self.image_height, self.image_width, c = self.cv_image.shape  # Get image dimensions
            self.image_received = 1  # Set the flag to indicate that color image is received

    def dep_image_callback(self, dep_image):
        # Callback function for the depth image subscriber

        self.depth_array = np.frombuffer(dep_image.data, dtype=np.uint16).reshape((dep_image.height, dep_image.width))  # Convert depth image to depth array

        if self.dep_received == 0:  # If depth image is not received before
            self.depy, self.depx = dep_image.height, dep_image.width  # Get depth image dimensions
            self.dep_received = 1  # Set the flag to indicate that depth image is received

    def cleanup(self):
        # Cleanup method called when the node is shutting down

        self.status.publish('Node killed successfully')  # Publish status message

if __name__ == "__main__":
    tracker_node()  # Create an instance of the tracker_node class and start the node