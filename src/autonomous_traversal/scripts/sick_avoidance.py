#!/usr/bin/python3
import rospy
import time
import math
import threading
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import String
import numpy as np
from masks import cardiod


class SickAvoider:
    def __init__(self):
        self.np_ranges = None
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.call_back)
        self.pub = rospy.Publisher("sick_cmd", String, queue_size=10)
        self.p = rospy.get_param('laser_max', 32)
        self.x = self.y = 0
        self.direction = self.magnitude = 0
        self.thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
        self.cosines = np.cos(self.thetas)
        self.sines = np.sin(self.thetas)
        # self._lock = threading.Lock()

    def cardiod(self, a, thetas):
        return a*(1+np.cos(thetas))

    def nomask(self, a, thetas):
        return a

    def call_back(self, data):
        if data.header.frame_id == 'cloud_POS_000_DIST1':
            self.np_ranges = np.array(data.ranges)
            self.np_ranges[self.np_ranges>16] = self.p
            self.np_ranges[self.np_ranges==0] = self.p
            sliced_thetas = self.thetas[190:911]
            sliced_np_ranges = self.np_ranges[190:911]
            sliced_np_ranges = self.cardiod(sliced_np_ranges, sliced_thetas).astype('float32')
            sliced_np_ranges[sliced_np_ranges < 5.0] = -20.0
            self.x = np.sum(sliced_np_ranges*self.cosines[190:911])
            self.y = np.sum(sliced_np_ranges*self.sines[190:911])
            self.direction = math.degrees(np.arctan2(self.y,self.x))
            self.magnitude = (np.sqrt(np.square(self.x) + np.square(self.y))) / (self.p*np.pi) 
            rospy.logdebug("Magnitude: %f, Direction: %f",self.magnitude, self.direction)

            #Add publisher after testing


if __name__ == '__main__':
    rospy.init_node("sick_avoidance")
    SickAvoider()
    rospy.spin()
            
