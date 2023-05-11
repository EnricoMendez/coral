#!/usr/bin/env python3
import tensorflow
import cv2
import numpy as np
#import os
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

class object_cls_node():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ### Suscriber
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        
        ### Publishers
        self.object_num_pub = rospy.Publisher("object_num", Int32, queue_size=1)
        
        ### Constants
        self.confidence_threshold = 0.75
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # list all packages, equivalent to rospack list
        rospack.list() 
        # get the file path for this ros pkg
        pkg_path = str(rospack.get_path('object_classification'))
        model_path = pkg_path + '/src/model_class_object_nuevas.h5'
        self.model = tensorflow.keras.models.load_model(model_path, compile=False)
        ### Variables
        self.part_num = 0
        self.confidence_score = 0
        self.image_received = 0
        self.bridge_object = CvBridge() # create the cv_bridge object
        self.cv_image = 0

        ###********** INIT NODE **********###  
        r = rospy.Rate(10)
        print('initialized node')

        while not rospy.is_shutdown():
            if self.image_received == 0:
                print('Image not received')
                continue
            self.image_processing()
            self.publish()
            r.sleep()

    def object_class(self,image):
        # Predict class
        prediction = self.model.predict(image, verbose=0)
        # Obtain value with higher confidence score
        index = np.argmax(prediction)
        self.confidence_score = prediction[0][index]
        # Predict class
        if self.confidence_score >= self.confidence_threshold:
            if index == 0:
                self.part_num = 1
            if index == 1:
                self.part_num = 2
            if index == 2:
                self.part_num = 3
            if index == 3:
                self.part_num = 4
            if index == 4:
                self.part_num = 5
            if index == 5:
                self.part_num = 6
            if index == 6:
                self.part_num = 7
        else:
            self.part_num = 0

    def image_processing(self):
        # Resize the raw image into (224-height,224-width) pixels
        image = self.cv_image
        image = cv2.resize(image, (256, 256), interpolation=cv2.INTER_AREA)
        # Change brightness and contrast of image
        image = cv2.addWeighted(image, 3., image, 0., 1.) 
        # Make the image a numpy array and reshape it to the models input shape.
        image = np.asarray(image, dtype=np.float32).reshape(1, 256, 256, 3)
        # Normalize the image array
        image = (image / 127.5) - 1
        self.object_class(image)

    def publish(self):
        self.object_num_pub.publish(self.part_num)
        # os.system('clear') 
        # print('Part number: ',self.part_num)
        # print('Confidence: ', self.confidence_score)

    def camera_callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e) 
        if self.image_received == 0: 
            self.image_received = 1

    def cleanup(self):    
        print('Node killed successfully')

if __name__ == "__main__":  
    rospy.init_node('object_class_node', anonymous=True)
    object_cls_node() 