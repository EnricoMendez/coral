#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from gtts import gTTS
import os


class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # Subscribers
        rospy.Subscriber("/zimmer/status", String, self.zimmer)  # Subscribe to Zimmer status topic
        rospy.Subscriber("/zimmer/inventory", Int32MultiArray, self.inventory)  # Subscribe to Zimmer inventory topic
        rospy.Subscriber("/move/status", String, self.move)  # Subscribe to move status topic
        rospy.Subscriber("/hand_track/status", String, self.hands)  # Subscribe to hand tracking status topic
        rospy.Subscriber("/object_class/status", String, self.object)  # Subscribe to object classification status topic
        rospy.Subscriber("/voice_recognition/status", String, self.voice)  # Subscribe to voice recognition status topic
        rospy.Subscriber("/arduino_commands", Int32, self.arduino)  # Subscribe to Arduino commands topic

        # Variables
        self.zimmer_msg = ['', '', '', '', '', '']  # Zimmer status messages
        self.move_msg = ['', '', '', '', '', '']  # Move status messages
        self.hands_msg = ['', '', '', '', '', '']  # Hand tracking status messages
        self.voice_msg = ['', '', '', '', '', '']  # Voice recognition status messages
        self.object_msg = ['', '', '', '', '', '']  # Object classification status messages
        self.arduino_msg = ['', '', '', '', '', '']  # Arduino commands
        self.inventory_msg = []  # Inventory

        # Constants
        self.capacity = [0, 1, 1, 2, 2, 2, 1, 1]  # Capacity of each inventory piece

        # Main loop
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            os.system('clear')
            self.print_msg()  # Print status messages
            r.sleep()

    def message_sort(self, msg, message_list):
        # Sort and update the message list with the latest message
        message_list = [message_list[1], message_list[2], message_list[3], message_list[4], message_list[5], msg]
        return message_list

    def talk(self, msg):
        # Convert the message to speech and play it
        tts = gTTS(msg, lang='en-us')
        tts.save('msg.mp3')
        os.system('mpg123 msg.mp3 > /dev/null 2>&1')

    def zimmer(self, data):
        self.zimmer_msg = self.message_sort(data.data, self.zimmer_msg)
        answer = data.data
        self.talk(answer)  # Convert Zimmer status message to speech

    def inventory(self, data):
        self.inventory_msg = data.data  # Update inventory message

    def move(self, data):
        self.move_msg = self.message_sort(data.data, self.move_msg)  # Update move status message

    def hands(self, data):
        self.hands_msg = self.message_sort(data.data, self.hands_msg)  # Update hand tracking status message

    def object(self, data):
        self.object_msg = self.message_sort(data.data, self.object_msg)  # Update object classification status message

    def voice(self, data):
        self.voice_msg = self.message_sort(data.data, self.voice_msg)  # Update voice recognition status message

    def arduino(self, data):
        self.arduino_msg = self.message_sort(str(data.data), self.arduino_msg)  # Update Arduino commands message

    def print_msg(self):
        # Print Zimmer status messages
        print('Zimmer node status:')
        for element in self.zimmer_msg:
            print('\t' + element)

        # Print voice recognition status messages
        print('Voice recognition node status:')
        for element in self.voice_msg:
            print('\t' + element)

        # Print object classification status message
        print('Object classification node status:')
        print('\t' + self.object_msg[5])

        # Print hand tracking status message
        print('Hands tracking node status:')
        print('\t' + self.hands_msg[5])

        # Print Arduino commands
        print('Arduino commands:')
        for i in range(3, len(self.arduino_msg)):
            print('\t' + self.arduino_msg[i])

        # Print move status messages
        print('Robot control node status:')
        for element in self.move_msg:
            print('\t' + element)

        # Print inventory
        print('Inventory:')
        for i in range(1, len(self.inventory_msg)):
            print('\tPiece ' + str(i) + ': ' + str(self.inventory_msg[i]) + '/' + str(self.capacity[i]))

    def cleanup(self):
        print('Node killed successfully')


if __name__ == "__main__":
    rospy.init_node('Supervisor', anonymous=False)
    supervisor = Supervisor()
