#!/usr/bin/python3
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np

rospy.init_node("tester", anonymous=True)
pub = rospy.Publisher("test_top", LaserScan, queue_size=10)
ls_1_og_input = LaserScan()
ls_1_og_input.ranges = np.zeros(1101).tolist()
ls_2_og_input = LaserScan()
ls_2_og_input.ranges = np.zeros(1101).tolist()

def ls_callback(data):
    global ls_og_input, ls_1_og_input, ls_2_og_input
    if data.header.frame_id == 'cloud_POS_000_DIST1':
        ls_1_og_input = data
        join()
    if data.header.frame_id == 'cloud_POS_000_DIST2':
        ls_2_og_input = data
        join()

def join():
    global ls_og_input, ls_1_og_input, ls_2_og_input
    ls_og_input = LaserScan()
    ls_og_input = ls_1_og_input
    ls_og_input.header.frame_id = 'cloud'
    ls_og_input.ranges = (np.array(ls_1_og_input.ranges) + np.array(ls_2_og_input.ranges)).tolist()
    pub.publish(ls_og_input)

def listener():
    rospy.Subscriber("scan", LaserScan, ls_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
