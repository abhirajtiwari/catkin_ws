#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rtabmap_ros.srv import *
import tf

rospy.init_node('vo_restarter.py', anonymous=True)

last_odom = Odometry()
last_imu = Imu()

def reset_odom_to(p, o):
    rospy.logdebug('Restarting Odometry')
    rospy.wait_for_service('reset_odom_to_pose')
    try:
        reset = rospy.ServiceProxy('reset_odom_to_pose', ResetPose)
        eu = tf.transformations.euler_from_quaternion((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        reset(o.pose.pose.x, o.pose.pose.y, o.pose.pose.z, eu[0], eu[1], eu[2])
        rospy.logdebug('Service reset successfull')
    except rospy.ServiceException, e:
        rospy.logfatal('Service call to restore vo failed')

def store_last_odom(data):
    global last_imu, last_odom
    if data.pose.covariance[0] == 9999.0:
        reset_odom_to(last_imu, last_odom)
    else:
        last_odom = data

def store_last_orientation(data):
    global last_imu
    last_imu = data
    
def main():
    rospy.Subscriber('/visual_odom', Odometry, store_last_odom)
    rospy.Subscriber('/phone/imu/data', Imu, store_last_orientation)
    rospy.spin()


if __name__ == '__main__':
    main()

