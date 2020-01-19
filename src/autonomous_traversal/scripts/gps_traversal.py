#!/usr/bin/python3
import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion


class GPSTraversal:
    def __init__(self):
        self.fix_val = None
        self.imu_val = None
        self.heading_diff = None
        self.bearing = None
        self.dist = None
        self.endcoods = None
        self.turn_gear = 10
        self.imu_sub = rospy.Subscriber("imu_data/raw", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("fix", NavSatFix, self.fix_callback)
        self.main()

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
            
    def fix_callback(data):
        self.fix_val = [data.latitude, data.longitude]
        if self.endcoods != None and self.fix_val != None:
            self.bearing, self.dist = self.get_heading(self.fix_val, self.endcoods)

    def get_heading(self, start, end):
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
    
    def match_head(self):
        set_gear(self.heading_diff)
        if self.heading_diff<180 :
            #turn anticlockwise add pub
        elif self.heading_diff>=180:
            #Turn Clockwise add pub

