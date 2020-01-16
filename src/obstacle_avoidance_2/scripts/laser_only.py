#!/usr/bin/python3
#from __future__ import print_function
import rospy
import time
import math
import threading 
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
from masks import cardiod, nomask
from tf.transformations import euler_from_quaternion
from masks import cardiod
import publish
from publish import ellipticalDiscToSquare, brute_stop, get_heading, match_head, forward, clockwise, anticlockwise
from checkclear import check_clear


global endcoods, precoods, imu_heading, np_ranges, sines, cosines, bearing, dist, heading_diff
np_ranges=np.zeros(1)
imu_heading=0
bearing=0.0
dist=0.0
heading_diff=0.0
precoods = [0, 0]
#endcoods = [13.3480430, 74.7919279]
endcoods = [13.3475449, 74.7920938] #home
#endcoods = [13.3480048, 74.7918997] 
aligner = 360 - 0
s=1
rospy.init_node("obs_av", anonymous=True, disable_signals=True)
pub = rospy.Publisher("masked", LaserScan, queue_size=10)
#global lock
#lock=threading.Lock()
def wait():
    global s
    while (s<=0):
        pass
    s-=1
def signal():
    global s
    s+=1

#Inputs from various sensors
free_ob = True
ls_og_input = LaserScan()
ls_1_og_input = LaserScan()
ls_1_og_input.ranges = np.zeros(1101).tolist()
ls_2_og_input = LaserScan()
ls_2_og_input.ranges = np.zeros(1101).tolist()
imu_og_input = Imu()
fix_og_input = NavSatFix()
masked_laser = LaserScan()

#Callbacks
def ls_callback(data):
    wait()
    global ls_og_input, ls_1_og_input, ls_2_og_input
    if data.header.frame_id == 'cloud_POS_000_DIST1':
        ls_1_og_input = data
    if data.header.frame_id == 'cloud_POS_000_DIST2':
        ls_2_og_input = data
    join()
    signal()

def join():
    global free_ob
    # while not free_ob:
    #     continue
    # free_ob = False
    global ls_og_input, ls_1_og_input, ls_2_og_input
    ls_og_input = LaserScan()
    ls_og_input = ls_1_og_input
    ls_og_input.header.frame_id = 'cloud'
    # free_ob = True
    # ls_og_input.ranges = (np.array(ls_1_og_input.ranges) + np.array(ls_2_og_input.ranges)).tolist()

def imu_callback(msg):
    global bearing, dist, heading_diff, imu_heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = math.degrees(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    imu_heading = 360 - yaw
    heading_diff = imu_heading - bearing
    if heading_diff < 0:
        heading_diff = heading_diff + 360

def fix_callback(data):
    global bearing, dist
    
    precoods = [data.latitude, data.longitude]
    
    bearing, dist = get_heading(precoods, endcoods)
    

#constant matrices for numpy calculations
thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
cosines = np.cos(thetas)
sines = np.sin(thetas)
ori_card=np.ones(1101)*8

def mask():
    global masked_laser, ls_og_input, thetas, free_ob
    global imu_heading, ls_og_input, precoods, np_ranges, sines, cosines, ori_card, heading_diff
    x=0
    y=0
    
    time.sleep(0.2)
    wait()
    # if not free_ob:
    #     continue
    # free_ob = False
    masked_laser = ls_og_input
    np_ranges = np.array(ls_og_input.ranges)
    
    if np_ranges.shape[0] != 0:
        p = 32
        np_ranges[np_ranges>16] =p
        #np_ranges[np_ranges<5] =-20             
        sliced_thetas = thetas[190:911]
        sliced_np_ranges = np_ranges[190:911]
        #print(sliced_np_ranges[sliced_np_ranges==32].shape)
        # sliced_while True:np_ranges[sliced_np_ranges == 0] = 16
        #print(np_ranges.shape, sliced_np_ranges.shape, sliced_thetas.shape)
        
        sliced_np_ranges = cardiod(sliced_np_ranges, sliced_thetas).astype('float32')
        sliced_np_ranges[sliced_np_ranges < 5.0] = -20.0
        # sliced_np_ranges = sliced_np_ranges/(np.pi*p) #
        # print(sliced_np_ranges.tolist())
        masked_laser.angle_max = np.pi/2
        masked_laser.angle_min = -(np.pi/2)
        masked_laser.ranges = sliced_np_ranges.tolist()
        masked_laser.intensities = []
        # x = np.sum(np_ranges*cosines)
        # y = np.sum(np_ranges*sines)
        x = np.sum(sliced_np_ranges*cosines[190:911])
        y = np.sum(sliced_np_ranges*sines[190:911])
        magnitude = (np.sqrt(np.square(x) + np.square(y))) / (p*np.pi) 
        direction = math.degrees(np.arctan2(y,x))
        print ("Magnitude",magnitude,"Direction", direction)

        rospy.logdebug("Magnitude- "+ str(magnitude) + " Direction- " + str(direction))
        pub.publish(masked_laser)

        print("input",x,y)
        rospy.logdebug("Beforex-"+str(x)+ "y-"+str(y))
        print("Heading diff",heading_diff)
        #print(check_clear(np_ranges , ori_card, 1.0, sines, cosines))

        # x_comps = np_ranges*cosines
        # y_comps = np_ranges*sines
        # l_ang = np.arctan2(np.sum(y_comps[]), np.sum(x_comps[]))
        # r_ang = np.arctan2(np.sum(y_comps[]), np.sum(x_comps[]))

        right_part = np_ranges[187:193] #12 is approx middle
        left_part = np_ranges[-193:-187]

        if abs(direction) < 10 and check_clear(np_ranges , ori_card, 1.0, sines, cosines):
            print('1')
            align(20)
        # elif (90 <= heading_diff <= 180 and direction > -20 ) or (180 <= heading_diff <= 270 and direction < 20 ):
        # elif (90 <= heading_diff <= 180 and 85 <= l_ang <= 95 and ) or (180 <= heading_diff <= 270 and -95 <= r_ang <= -85 and ):
        elif (90 <= heading_diff <= 180 and np.mean(left_part) >= 6) or (180 <= heading_diff <= 270 and np.mean(right_part) >= 6):
            align(2.5)
        else:
            ellipticalDiscToSquare(-y, x)
        
    signal()
        # free_ob = True

def align(buffer):
    global heading_diff, imu_heading, bearing, precoods, endcoods,np_ranges
    print("Precoods",precoods, bearing)
    print (heading_diff)
    while abs(heading_diff) >= buffer:
        print("Heading",imu_heading,"Bearing",bearing,"Difference",heading_diff),
        right_part = np_ranges[187:193] #12 is approx middle
        left_part = np_ranges[-193:-187]
        if (heading_diff <=180 and np.mean(left_part) <= 8) or (180<heading_diff and np.mean(right_part) <= 8):
            print("exiting.............")
            return
        match_head(precoods, endcoods, heading_diff)

try:
    def main():
        global dist

        rospy.Subscriber("scan", LaserScan, ls_callback)
        rospy.Subscriber("imu_data/raw", Imu, imu_callback)
        rospy.Subscriber("fix", NavSatFix, fix_callback)
        time.sleep(1)
        #while True:
        mask()
        align(2.5)                
        while True:
            print("\n") 
            print("Distance remaining",dist)
            if dist <= 3:
                print("Reached.")
                brute_stop()
                return
            else:
                mask()
        
        rospy.spin()

finally:
    brute_stop()
if __name__ == '__main__':
    main()



