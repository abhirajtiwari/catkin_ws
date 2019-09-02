#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import NavSatFix

rospy.init_node('gps_correcter.py', anonymous=True)
pub = rospy.Publisher('/phone/fix', NavSatFix, queue_size=10)

def correct(data):
    data.header.frame_id = '/world'
    data.altitude = 400
    data.header.stamp = rospy.Time.now()
    pub.publish(data)

def main():
    rospy.Subscriber('/fix', NavSatFix, correct)
    rospy.spin()

if __name__=='__main__':
    main()
