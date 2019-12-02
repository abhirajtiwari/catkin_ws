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
from publish import ellipticalDiscToSquare, brute_stop, get_heading, match_head
from checkclear import check_clear


global endcoods, precoods, imu_heading, np_ranges, sines, cosines
np_ranges=0
imu_heading=0
precoods = [0, 0]
endcoods = [13.3480237, 74.7921562]
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
    global imu_og_input
    imu_og_input = msg
    global imu_heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = math.degrees(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    imu_heading = 360 - yaw

def fix_callback(data):
    global fix_og_input
    fix_og_input = data

#constant matrices for numpy calculations
thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
cosines = np.cos(thetas)
sines = np.sin(thetas)

def mask():
    global masked_laser, ls_og_input
    global thetas
    global free_ob
    while True:
        time.sleep(0.2)
        wait()
        # if not free_ob:
        #     continue
        # free_ob = False
        masked_laser = ls_og_input
        np_ranges = np.array(ls_og_input.ranges)
        if np_ranges.shape[0] != 0:
            p = 16
            np_ranges[np_ranges>p] = p #
            sliced_thetas = thetas[190:911]
            sliced_np_ranges = np_ranges[190:911]
            # sliced_np_ranges[sliced_np_ranges == 0] = 16
            print(np_ranges.shape, sliced_np_ranges.shape, sliced_thetas.shape)
            sliced_np_ranges = cardiod(sliced_np_ranges, sliced_thetas).astype('float32')
            sliced_np_ranges = sliced_np_ranges/(2*p) #
            # print(sliced_np_ranges.tolist())
            masked_laser.angle_max = np.pi/2
            masked_laser.angle_min = -(np.pi/2)
            masked_laser.ranges = sliced_np_ranges.tolist()
            # x = np.sum(np_ranges*cosines)
            # y = np.sum(np_ranges*sines)
            x = np.sum(sliced_np_ranges*cosines[190:911])
            y = np.sum(sliced_np_ranges*sines[190:911])
            print (np.sqrt(np.square(x) + np.square(y)), math.degrees(np.arctan2(y,x))*8)
            pub.publish(masked_laser)
            ellipticalDiscToSquare(x,y)
        signal()
        # free_ob = True


def main():
    global imu_heading, masked_laser, ls_og_input, precoods, np_ranges, sines, cosines

    rospy.Subscriber("scan", LaserScan, ls_callback)
    rospy.Subscriber("imu_data/raw", Imu, imu_callback)
    rospy.Subscriber("fix", NavSatFix, fix_callback)
    mask()
    while True:
        bearing, dist = get_heading(precoods, endcoods)
        heading_diff = imu_heading - bearing
        if abs(heading_diff) >= 90:
            while abs(heading_diff)>=2.5:
                match_head(precoods, endcoods, heading_diff)
                bearing, dist = get_heading(precoods, endcoods)
                heading_diff = imu_heading - bearing
                print("Heading",imu_heading,"Bearing",bearing,"Difference",heading_diff),
        elif dist <= 3:
            print("Reached. Distance remaining",dist)
            return
        elif check_clear(np_ranges, ori_card, 0.5, sines, cosines):
            mask()
    
    rospy.spin()

if __name__ == '__main__':
    main()
