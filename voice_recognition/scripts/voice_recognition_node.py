#!/usr/bin/env python3

import re
import time

import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
import serial
import tensorflow as tf
from std_msgs.msg import String


class voice_recognition():
    def __init__(self):
        
        ###******* INIT PUBLISHERS *******###  
        self.status_pub = rospy.Publisher('/voice_recognition/status',String,queue_size=10)
        self.command_pub = rospy.Publisher('/voice_commands',String,queue_size=10)

        rospy.Subscriber("/zimmer/status",String,self.change_interpreter)

        ### Constants
        self.commands = ['noise', 'take','bring','go','cancel']
        rospack = rospkg.RosPack()
        rospack.list() 
        pkg_path = str(rospack.get_path('voice_recognition'))
        self.commands_model_name = pkg_path + '/scripts/commands.tflite'
        self.commands_interpreter = tf.lite.Interpreter(model_path=self.commands_model_name)

        self.numbers_model_name = pkg_path + '/scripts/numbers.tflite'
        self.numbers_interpreter = tf.lite.Interpreter(model_path=self.numbers_model_name)
        self.interpreter = self.commands_interpreter
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.port_name = serial.Serial("/dev/ttyACM0", 9600)
        self.fs = 16000
        self.win = np.hamming(1024)
        self.nfft = 1024
        self.hop = self.nfft // 2
        self.time_per_sample = 2
        self.samples_per_record = int (3327)
        self.threshold = 0.60


        ### Variables
        self.inputs=[]
        self.data_save = []
        self.start_time = time.time()
        

        
        ### Main loop ###
        
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(.5)
        time.sleep(0.2)
        self.status_pub.publish('Node initialized')
        while not rospy.is_shutdown():
            while len(self.data_save) < self.samples_per_record:
                self.data_record()
            self.command_recognition()
            self.send_command()
            r.sleep()
    
    def prediction(self,input):
        # Asignar los datos de entrada al tensor de entrada del modelo
        input_data = np.expand_dims(input.astype('float32'), axis=0)
        self.interpreter.allocate_tensors()
        
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # Ejecutar el modelo
        self.interpreter.invoke()

        # Obtener los resultados del modelo
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        true = np.argmax(output_data[0])
        output = self.commands[true]
        confidence = output_data[0][true]
        return output, confidence

    def data_record(self):
        data = self.port_name.readline()
        data_str = data.decode()  # convierte bytes en una cadena
        # busca números enteros con signo
        numbers_only = re.findall(r'[-+]?\d+', data_str)
        for number in numbers_only:
            self.data_save += [number]
    
    def command_recognition(self):
        data_array = np.array(self.data_save, dtype=np.float64) # convertir data_save a un vector NumPy de tipo float64
        spec, _, _, _ = plt.specgram(data_array, NFFT=self.nfft, Fs=self.fs, window=self.win, noverlap=self.hop, mode='magnitude')
        plt.close()
        self.data_save = []
        spec = np.expand_dims(spec, axis=0)
        self.voice_command,self.confidence = self.prediction(spec)

    def send_command(self):
        msg = str(self.voice_command)+' recognize with '+ str(round(self.confidence,3)) + ' of confidence'
        self.status_pub.publish(msg)
        if self.confidence > self.threshold:
            self.status_pub.publish('Command published')
            self.command_pub.publish(self.voice_command)
        time.sleep(1)

    def change_interpreter(self,data):
        msg = data.data

        if msg == 'Bring routine' or msg == 'Take routine' :
            self.interpreter = self.numbers_interpreter
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            self.commands = ['noise','cancel','one','two','three','four','five','six','seven']
            self.status_pub.publish('Changing to numbers interpreter')
        elif msg == 'Say go to open the gripper'or msg == 'Routine canceled' or msg == 'Say go to close the gripper':
            self.interpreter = self.commands_interpreter
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            self.commands = ['noise', 'take','bring','go','cancel']
            self.status_pub.publish('Changing to commands interpreter')

    def cleanup(self) :
        self.port_name.close()
        self.status_pub.publish("Port closed")
        self.status_pub.publish('Node killed')

if __name__ == "__main__":
    rospy.init_node('voice_recognition',anonymous=False)
    voice_recognition()