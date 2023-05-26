#!/usr/bin/env python3
import tensorflow
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import time

class object_cls_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Subscriber
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback) 
        
        ### Publishers
        self.object_num_pub = rospy.Publisher("object_num", Int32, queue_size=1)
        self.status = rospy.Publisher("/object_class/status", String, queue_size=1)
        
        ### Constants
        self.confidence_threshold = 0.75
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # List all packages, equivalent to rospack list
        rospack.list() 
        # Get the file path for this ROS package
        pkg_path = str(rospack.get_path('object_classification'))
        model_path = pkg_path + '/scripts/imageclassifier0.h5'
        self.model = tensorflow.keras.models.load_model(model_path, compile=False)
        
        ### Variables
        self.part_num = 0 # Part number variable
        self.confidence_score = 0 # Confidence score of prediction
        self.image_received = 0 # Flag for image reception
        self.bridge_object = CvBridge() # Create the cv_bridge object
        self.cv_image = 0

        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        self.status.publish('Initialized node')

        while not rospy.is_shutdown():
            if self.image_received == 0:
                self.status.publish('Image not received')
                continue
            self.image_processing()
            self.publish()
            r.sleep()

    def object_class(self, image):
        # Predict class
        time.sleep(0.5)
        prediction = self.model.predict(image, verbose=0)
        # Obtain value with higher confidence score
        index = np.argmax(prediction)
        self.confidence_score = prediction[0][index]
        # Predict class
        if self.confidence_score >= self.confidence_threshold:
            # Assign values between 1 and 7
            if 0 <= index <= 6:
                self.part_num = index + 1
        else:
            self.part_num = 8

    def image_processing(self):
        # Resize the raw image into (224-height, 224-width) pixels
        image = self.cv_image
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
        # Change brightness and contrast of image
        image = cv2.addWeighted(image, 3., image, 0., 1.) 
        # Make the image a numpy array and reshape it to the model's input shape.
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
        # Normalize the image array
        image = (image / 127.5) - 1
        self.object_class(image)

    def publish(self):
        self.object_num_pub.publish(self.part_num)
        msg = 'Object identified: ' + str(self.part_num)
        self.status.publish(msg)

    def camera_callback(self, data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.status.publish(e)  
        if self.image_received == 0: 
            self.image_received = 1

    def cleanup(self):    
        self.status.publish('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('object_class_node', anonymous=True)
    object_cls_node()