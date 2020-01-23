#!/usr/bin/python
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
        self.endcoods = [0,0]
        self.priority = 0
        self.hard_turn_min = 90
        self.turn_gear = 10
        self.imu_sub = rospy.Subscriber("imu_data/raw", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("fix", NavSatFix, self.fix_callback)

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
            self.turn_gear=2
        elif abs(self.heading_diff) <= 10:
            self.turn_gear=2
        else:
            self.turn_gear=7

    def match_head_cmds(self):
        self.set_gear()
        if self.hard_turn_min <= self.heading_diff < 180 :
            return str(self.priority)+ "1"+ " 90"+ str(self.turn_gear)
        elif -180 <= self.heading_diff <= -90:
            return str(self.priority)+ " 1 " + "-90 " + str(self.turn_gear)
        else: return str(self.priority)+ " 1 "+ str(int(self.heading_diff))+ " "+ str(7)

    def align(self,buf):
        if self.heading_diff is not None:
            rospy.logdebug("Aligning rover %f",self.heading_diff)
        try:
            ccserviceProxy = rospy.ServiceProxy('check_clear', ClearService)
        except:
            side_clear = 1
        # while abs(self.heading_diff) >= buf:
        while self.heading_diff is None:
            continue
        try:
            side_clear = ccserviceProxy(90 if (180 >= self.heading_diff >= self.hard_turn_min) else -90 if (-180 <= self.heading_diff <= -self.hard_turn_min) else self.heading_diff)
        except:
            side_clear = 1
        
        if side_clear == 1:
            rospy.logdebug(self.match_head_cmds())

if __name__ == '__main__':
    pub = rospy.Publisher("soft_gps_cmd", String, queue_size = 10)
    rospy.init_node("sof_gps_traversal", log_level=rospy.DEBUG, disable_signals=True)
    ob = GPSTraversal()

    while True:
        try:
            pub.publish(ob.align(2.5))
        except KeyboardInterrupt:
            break


