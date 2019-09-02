#!/usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry

rospy.init_node('odom_correcter.py', anonymous=True)
pub = rospy.Publisher('/odometry/gps', Odometry, queue_size=10)

def correct(data):
    data.child_frame_id = 'base_link'
    data.header.stamp = rospy.Time.now()
    pub.publish(data)

def main():
    rospy.Subscriber('/odometry/pre_gps', Odometry, correct)
    rospy.spin()

if __name__=='__main__':
    main()
