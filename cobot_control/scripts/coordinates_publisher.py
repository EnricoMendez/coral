#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray


rospy.init_node('coordinates_publisher')
mi_array = Float32MultiArray()
mi_array.data = [0.9, .9, .9]
pub = rospy.Publisher('/coordinates_coral', Float32MultiArray, queue_size=10)
while not rospy.is_shutdown():
    pub.publish(mi_array)
    rospy.sleep(1)


