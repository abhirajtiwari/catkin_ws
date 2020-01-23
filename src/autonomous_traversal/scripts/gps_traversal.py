#!/usr/bin/python
import rospy
import numpy as np
import math
import pyproj
from tf.transformations import euler_from_quaternion
from autonomous_traversal.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class GPSTraversal:
    def __init__(self):
        self.fix_val = None
        self.imu_val = None
        self.heading_diff = None
        self.bearing = None
        self.dist = None
        self.endcoods = [0,0]
        self.turn_gear = 10
        self.imu_sub = rospy.Subscriber("/imu_data/raw", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.fix_callback)
        self.pub = rospy.Publisher("gps_cmd", String, queue_size = 10)

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
            if self.heading_diff < 0:
                self.heading_diff = self.heading_diff + 360

    def fix_callback(self, data):
        self.fix_val = [data.latitude, data.longitude]
        if self.endcoods != None and self.fix_val != None:
            self.bearing, self.dist = self.get_heading(self.fix_val, self.endcoods)

    def get_heading(self, start, end):
        g = pyproj.Geod(ellps='WGS84')
        (az12, az21, dist) = g.inv(start[1], start[0], end[1], end[0])
        if az12<0:
            az12=az12+360
        return az12,dist 

    def set_gear(self, head_diff):
        if abs(head_diff) <= 30 or abs(head_diff) >= 330:
            self.turn_gear=4
        elif abs(head_diff) <= 20 or abs(head_diff) >= 340:
            self.turn_gear=2
        elif abs(head_diff) <= 10 or abs(head_diff) >= 350:
            self.turn_gear=2
        else:
            self.turn_gear=7

    def match_head_cmds(self):
        self.set_gear(self.heading_diff)
        if self.heading_diff < 180 :
            return '0' + ' 1' + ' 90 ' + str(self.turn_gear)
        elif self.heading_diff >= 180:
            return '0' + ' 1' + ' -90 ' + str(self.turn_gear)

    def align(self,buf):
        if self.heading_diff is not None:
            rospy.logdebug("Heading: %f",self.heading_diff)
            try:
                ccserviceProxy = rospy.ServiceProxy('check_clear', ClearService)
            except:
                side_clear = 1
            while abs(self.heading_diff) >= buf:
                rospy.logdebug("Aligning rover %f",self.heading_diff)
                try:
                    side_clear = ccserviceProxy(-90 if (180 >=self.heading_diff >= 0) else 90)
                    rospy.logdebug("side_clear: %d", side_clear)
                except:
                    side_clear = 1
                if side_clear != 1 : 
                    break
                self.pub.publish ( self.match_head_cmds() )

if __name__ == '__main__':
    rospy.init_node("gps_traversal", log_level = rospy.DEBUG, disable_signals=True)
    ob = GPSTraversal()
    while True:
        try:
            ob.align(10)
        except KeyboardInterrupt:
            break

