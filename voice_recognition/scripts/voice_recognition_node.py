#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tensorflow as tf
import numpy as np
import re
import serial
import numpy as np
import matplotlib.pyplot as plt
import time


class voice_recognition():
    def __init__(self):
        
        ###******* INIT PUBLISHERS *******###  
        self.status_pub = rospy.Publisher('/voice_recognition/status',String,queue_size=10)
        self.command_pub = rospy.Publisher('/voice_commands',String,queue_size=10)

        ### Constants
        rospy.on_shutdown(self.cleanup)
        self.commands = ['noise','go', 'take','bring','cancel','one','two','three','four','five','six','seven']
        self.model_name = 'command_model.tflite'
        self.interpreter = tf.lite.Interpreter(model_path=self.model_name)
        self.self.commands=rospy.Publisher("voice_commands",String,queue_size=10)
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.port_name = serial.Serial("/dev/ttyACM0", 9600)
        self.fs = 16000
        win = np.hamming(1024)
        nfft = 1024
        self.hop = nfft // 2
        self.time_per_sample = 2
        self.samples_per_record = int (3327)
        self.threshold = 0.9

        ### Variables
        self.inputs=[]
        self.data_save = []
        self.start_time = time.time()
        

        
        ### Main loop ###
        
        r = rospy.Rate(10)
        self.status_pub.publish('node init')
        while not rospy.is_shutdown():
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
        return self.commands[true],output_data[0][true]

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
        self.voice_command,self.confidence = self.prediction(spec,self.input_details,self.interpreter,self.output_details)

    def send_command(self):
        msg = str(self.voice_command)+' recognize with '+ str(round(self.confidence,3)) + 'of confidence'
        self.status_pub.publish(msg)
        if self.confidence > self.threshold:
            self.status_pub('Command published')
            self.command_pub.publish(self.voice_command)

    def cleanup(self):
        self.port.close()
        self.status_pub.publish("Port closed")
        self.status_pub.publish('Node killed')

if __name__ == "__main__":
    rospy.init_node('voice_recognition',anonymous=False)
    voice_recognition()