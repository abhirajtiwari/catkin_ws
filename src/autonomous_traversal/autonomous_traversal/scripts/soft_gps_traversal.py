#!/usr/bin/python3
import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from autonomous_traversal.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import pyproj

class GPSTraversal:
    def __init__(self):
        self.fix_val = None
        self.imu_val = None
        self.heading_diff = None
        self.bearing = None
        self.dist = None
        #self.endcoods = [13.349719, 74.791403]
        #self.endcoods = [13.3475449, 74.7920938] #home
        #self.endcoods = [13.348080, 74.792655] #parking lot N
        #self.endcoods = [13.3476127, 74.7928378] #kc gate 2 deeper
        #self.endcoods = [13.347486, 74.792730] #kc gate
        #self.endcoods = [13.3501680, 74.7911333]
        #self.endcoods = [13.3502136, 74.7912489] #irc hill
        #self.endcoods = [13.3496395, 74.7915024] #irc home
        self.endcoods = [13.3496875, 74.7916549]#ditch east
        #self.endcoods = [13.34961833, 74.79167999] #shade east
        self.priority = 0
        self.hard_turn_min = 90
        self.turn_gear = 6
        self.imu_sub = rospy.Subscriber("imu_data/raw", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("fix", NavSatFix, self.fix_callback)
        self.pub = rospy.Publisher("soft_gps_cmd", String, queue_size = 10)


    def imu_callback(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        aligner = 360

        yaw = math.degrees(yaw)
        if yaw < 0:
            yaw += 360
        yaw = (yaw + aligner) % 360

        self.imu_val = 360 - yaw
        if self.bearing != None:
            self.heading_diff = self.imu_val - self.bearing
            if self.heading_diff < -180:
                self.heading_diff += 360 #COnverting Difference to 180 to 180


    def fix_callback(self, data):
        self.fix_val = [data.latitude, data.longitude]
        if self.endcoods != None and self.fix_val != None:
            self.bearing, self.dist = self.get_heading()

    def get_heading(self):
        g = pyproj.Geod(ellps='WGS84')
        (az12, az21, dist) = g.inv(self.fix_val[1], self.fix_val[0], self.endcoods[1], self.endcoods[0])
        if az12<0:
            az12=az12+360
        return az12,dist 

    def set_gear(self):
        if abs(self.heading_diff) <= 30:
            self.turn_gear=4
        elif abs(self.heading_diff) <= 20:
            self.turn_gear=3
        elif abs(self.heading_diff) <= 10:
            self.turn_gear=2
        else:
            self.turn_gear=7
    def send_cmds(self, pub_str):
        self.set_gear()
        rospy.logdebug("Motor Commands: %s",pub_str)
        self.pub.publish(pub_str)

    def match_head_cmds(self):
        if self.hard_turn_min <= self.heading_diff < 180 :
            self.priority = 1
            while abs(self.heading_diff)>=2.5:
                pub_str = str(self.priority)+ " 1"+ " 90 "+ str(self.turn_gear)
                self.send_cmds(pub_str)
        elif -180 <= self.heading_diff <= -self.hard_turn_min:
            self.priority = 1
            while abs(self.heading_diff)>=2.5:
                pub_str = str(self.priority)+ " 1"+ " -90 "+ str(self.turn_gear)
                self.send_cmds(pub_str)
        else:
            self.priority = 0
            self.send_cmds(str(self.priority)+ " 1 "+ str(int(self.heading_diff))+ " "+ str(7))

    def align(self,buf):
        if self.heading_diff is not None:
            rospy.logdebug("Aligning rover %f\tDistance Remaining %f", self.heading_diff, self.dist)
            
        try:
            ccserviceProxy = rospy.ServiceProxy('check_clear', ClearService)
        except:
            side_clear = 1
        # while abs(self.heading_diff) >= buf:
        while self.heading_diff is None:
            rospy.logdebug_once("No GPS!")
            continue

        rospy.logdebug("GPS Found")
        try:
            #side_clear = ccserviceProxy(90 if (180 >= self.heading_diff >= self.hard_turn_min) else -90 if (-180 <= self.heading_diff <= -self.hard_turn_min) else self.heading_diff).response
            side_clear = ccserviceProxy(self.heading_diff).response
        except:
            side_clear = 1
        rospy.logdebug("side_clear: %d \tHeading diff: %f", side_clear, self.heading_diff)
        
        if self.dist<=5:
            while True:
                pub_str = str(1)+ " 0"+ " 0 "+ str(self.turn_gear)
                self.send_cmds(pub_str)
        if side_clear == 1:
            self.match_head_cmds()
        else:
            self.pub.publish(str(0)+ " 1 "+ str(0)+ " "+ str(7))
            
if __name__ == '__main__':
    rospy.init_node("sof_gps_traversal", log_level=rospy.DEBUG, disable_signals=True)
    ob = GPSTraversal()
    ob.hard_turn_min=0
    ob.align(2.5)


    while True:
        try:
            ob.hard_turn_min=90
            ob.align(2.5)
        except KeyboardInterrupt:
            break


