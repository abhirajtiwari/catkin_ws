import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
from cardiod import cardiod

#Inputs from various sensors
ls_og_input = LaserScan()
ls_1_og_input = LaserScan()
ls_1_og_input.ranges = np.zeros(1101).tolist()
ls_2_og_input = LaserScan()
ls_2_og_input.ranges = np.zeros(1101).tolist()
imu_og_input = Imu()
fix_og_input = NavSatFix()

#Callbacks
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

def imu_callback(data):
    global imu_og_input
    imu_og_input = data

def fix_callback(data):
    global fix_og_input
    fix_og_input = data

#constant matrices for numpy calculations
thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
cosines = np.cos(thetas)
sines = np.sin(thetas)

    
