#!/usr/bin/python3
import rospy
import numpy as np
import math
import time

from std_msgs.msg import String
from gps_traversal import GPSTraversal

class JoyMux:
    '''
    Let all the data be arrays converted to string having 3 fields
    priority | r | theta | gear
    priority be just an extensible feature to be added later could be used for overriding
    '''
    def __init__(self):
        self.rs_data = None
        self.sick_data = None
        self.gps_data = None
        self.gear=5
        #self.prev_rs_data=None
        self.rs_sub = rospy.Subscriber('kinect_data', String, self.rs_callback)
        self.sick_sub = rospy.Subscriber('sick_cmd', String, self.sick_callback)
        self.gps_ob = GPSTraversal()
        self.pub=rospy.Publisher("auto_trav_cmd", String, queue_size=10)
        self.start()

    def rs_callback(self, data):
        self.rs_data = list(map(float, data.data.split()))

    def sick_callback(self, data):
        self.sick_data = list(map(float, data.data.split()))

    def send_cmd(self,u, v, gear):
                
        rospy.logdebug("u and v %f, %f",u,v)
        #v=-v #shitty way to solve
        def map1(x,in_min,in_max,out_min,out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        rang = ((u**2)+(v**2))**0.5
        u = map1 (u, -rang, rang, -1, 1)
        v = map1 (v, -rang, rang, -1, 1)
        u2 = u * u
        v2 = v * v
        twosqrt2 = 2.0 * (2.0)**(0.5)
        subtermx = 2.0 + u2 - v2
        subtermy = 2.0 - u2 + v2
        termx = subtermx + u * twosqrt2
        termx2 = subtermx - u * twosqrt2
        termy = subtermy + v * twosqrt2
        termy2 = subtermy - v * twosqrt2
        x = 0.5 * (termx)**(0.5) - 0.5 * (termx2)**(0.5)
        y = 0.5 * (termy)**(0.5) - 0.5 * (termy2)**(0.5)
        x = map1(x, -1, 1, -8000, 8000) + 8000
        y = 8000 + map1(y, -1, 1, -8000, 8000)
        rospy.logdebug("Joystick x: "+str(x)+ " y: "+str(y)+ " Heading: " + str(self.gps_ob.heading_diff))
        # Publish this data to a decoder    
        self.pub.publish(str(int(x))+" "+str(int(y))+" "+str(self.gear))

    def start(self):

        while self.gps_ob.heading_diff is None:
            rospy.logdebug_once("Waiting for gps...")
            continue

        rospy.logdebug("Got GPS...")
        time.sleep(1)
        self.gps_ob.align(5) #First alignment

        while True:
            if self.gps_ob.dist <= 3:
                rospy.loginfo("REACHED!!")
                self.pub.publish(str(int(8000))+" "+str(int(8000))+" "+str(self.gear))
                break
            #Destroy degree data less than 5degs
            if self.gps_ob.heading_diff is not None:
                rospy.logdebug("Distance: %f Heading diff %f ",self.gps_ob.dist, self.gps_ob.heading_diff)
            if self.rs_data is not None: #rs_data
                self.rs_data[2] = 0 if (abs(self.rs_data[2]) <= 5) else self.rs_data[2]
                if self.rs_data[2] != 0: 
                    rospy.logdebug("Listening to rs")
                    self.send_cmd(-1*self.rs_data[1]*np.sin(np.radians(self.rs_data[2])), self.rs_data[1]*np.cos(np.radians(self.rs_data[2])), self.rs_data[3])
                    continue

            if  self.gps_ob.heading_diff is not None:
            	if(90<=self.gps_ob.heading_diff<=180 or 180<=self.gps_ob.heading_diff<=270):
                    rospy.logdebug("GPS ")
                    self.gps_ob.align(5)
                    if self.gps_ob.side_clear==1:

                        continue
            # if self.rs_data is not None: #Realsense
            #     if self.rs_data[2] == 90 :
            #         while self.rs_data[2] != 0:
            #             #send hard left turn
            #     elif self.rs_data[2] == -90:
            #         while self.rs_data!=0:
            #             #send hard right turn
 
            if self.sick_data is not None: #Sick data
                self.sick_data[2] = 0 if (abs(self.sick_data[2]) <= 9) else self.sick_data[2]
                if self.sick_data[2] != 0:
                    rospy.logdebug("Listening to sick")
                    #self.send_cmd(-1*self.sick_data[1]*np.sin(np.radians(self.sick_data[2])), self.sick_data[1]*np.cos(np.radians(self.sick_data[2])), self.sick_data[3])
                    self.send_cmd(-1*self.sick_data[2],self.sick_data[1],self.sick_data[3])
                    continue


            #GPS Data Correction
            self.send_cmd(8000, 16000, self.gear)

            # self.send_cmd( STRAIGHT BASICALLY )
            rospy.logdebug("Straight")


if __name__ == '__main__':
    rospy.init_node('joy_mux',log_level=rospy.DEBUG,anonymous=True,disable_signals=True)
    mux_obj=JoyMux()
    rospy.spin()

