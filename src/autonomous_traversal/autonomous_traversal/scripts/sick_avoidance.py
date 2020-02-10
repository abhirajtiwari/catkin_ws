#!/usr/bin/python3
import rospy
import time
import math
import threading
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np


from autonomous_traversal.srv import ClearService, ClearServiceResponse


class SickAvoider:
    def __init__(self):
        self.np_ranges = None
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.call_back)
        self.pub = rospy.Publisher("sick_cmd", String, queue_size=10)
        self.service = rospy.Service('check_clear', ClearService, self.clear_service_callback)
        self.p = rospy.get_param('laser_max', 32)
        self.x = self.y = 0
        self.direction = self.magnitude = 0
        self.thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
        self.cosines = np.cos(self.thetas)
        self.sines = np.sin(self.thetas)
        # self._lock = threading.Lock()

    def clear_service_callback(self,req):
        req.angle = 137.5 + req.angle
        mid = int(req.angle * 1101/275)
        buf = 8
        left = 0 if mid-buf<0 else mid-buf
        right = 1100 if mid+buf>1100 else mid+buf
        if self.np_ranges is not None:
            if mid<0 or mid>1100:
                return ClearServiceResponse(1)
            rospy.logdebug("%d, %d, mean %f", left, right, np.mean(self.np_ranges[left:right]))
            print (self.np_ranges[left:right])
            if np.amin(self.np_ranges[left:right]) < 6:	 #Tweakable param here
                rospy.logdebug("not clear %f", req.angle)
                return ClearServiceResponse(0)
            else: return ClearServiceResponse(1)
        else: return ClearServiceResponse(1)

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
            send = String()
            send.data = '0' + ' ' + str(self.x) + ' ' + str(self.y) + ' 5'
            self.pub.publish(send)


if __name__ == '__main__':
    rospy.init_node("sick_avoidance", log_level=rospy.DEBUG)
    SickAvoider()
    rospy.spin()
            
