#!/usr/bin/python3
import rospy
import numpy as np
import math
import time
import serial

from std_msgs.msg import String

from gps_traversal.py import GPSTraversal

class JoyMux:
    '''
    Let all the data be numpy arrays converted to string having 3 fields
    priority | r | theta
    priority be just an extensible feature to be added later could be used for overriding
    '''
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB', 115200)
        self.rs_data = None
        self.sick_data = None
        self.gps_data = None

        self.rs_sub = rospy.Subscriber('', String, queue_size = 10)
        self.sick_sub = rospy.Subscriber('sick_cmd', String, queue_size = 10)
        self.gps_ob = GPSTraversal()

        self.start()

    def rs_callback(self, data):
        self.rs_data = list(map(int, data.data.split()))

    def sick_callback(self, data):
        self.gps_data = list(map(int, data.data.split()))

    def ellipticalDisctoSquare(self, x, y):

    def start(self):
        
        #destroy turn values within 5degs
        self.rs_data[2] = 0 if (abs(self.rs_data[2]) <= 5) else self.rs_data[2]
        self.sick_data[2] = 0 if (abs(self.sick_data[2]) <= 5) else self.sick_data[2]

        #just before serial write fill these
        rospy.logdebug()


    

if __name__ == '__main__':
    rospy.init_node('joy_mux')
    JoyMux()
    rospy.spin()

