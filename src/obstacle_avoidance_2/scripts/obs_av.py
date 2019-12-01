import rospy
import time
import math
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
from masks import cardiod
from publish import ellipticalDiscToSquare, brute_stop, get_heading, match_head
from checkclear import check_clear

global endlat, endlon
endlat = 13.3480237
endlon = 74.7921562
rospy.init_node("obs_av", anonymous=True, disable_signals=True)
pub = rospy.Publisher("masked", LaserScan, queue_size=10)

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
    global ls_og_input, ls_1_og_input, ls_2_og_input
    if data.header.frame_id == 'cloud_POS_000_DIST1':
        ls_1_og_input = data
    if data.header.frame_id == 'cloud_POS_000_DIST2':
        ls_2_og_input = data
    join()

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

def imu_callback(data):
    global imu_og_input
    imu_og_input = data
    global imu_heading, aligner

    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw = radians(yaw)
    if yaw < 0:
        yaw += 360
    yaw = (yaw + aligner) % 360

    imu_heading = 360 - yaw
    # print(heading)

def fix_callback(data):
    global fix_og_input, pre_lat, pre_lon
    fix_og_input = data
    pre_lat = data.latitude
    pre_lon = data.longitude

#constant matrices for numpy calculations
thetas = np.arange(-2.39982771873, 2.39982771873, 0.00436332309619)
cosines = np.cos(thetas)
sines = np.sin(thetas)

def mask():
    global masked_laser, ls_og_input
    global thetas
    global free_ob
    while True:
        time.sleep(0.1)
        # if not free_ob:
        #     continue
        # free_ob = False
        masked_laser = ls_og_input
        np_ranges = np.array(ls_og_input.ranges)
        if np_ranges.shape[0] != 0:
            # np_ranges[np_ranges>16] = 16
            np_ranges = cardiod(np_ranges, thetas).astype('float32')
            masked_laser.ranges = np_ranges.tolist()
            x = np.sum(np_ranges*cosines)
            y = np.sum(np_ranges*sines)
            print (math.degrees(np.arctan2(y,x)))
            pub.publish(masked_laser)
        # free_ob = True


def main():
    global pre_lat, pre_lon, imu_heading
    rospy.Subscriber("scan", LaserScan, ls_callback)
    rospy.Subscriber("imu_data/raw", Imu, imu_callback)
    rospy.Subscriber("fix", NavSatFix, fix_callback)
    bearing, dist = get_heading(pre_lon, pre_lat, endlon, endlat)
    heading_diff = bearing - imu_heading
    if abs(heading_diff) >= 90:
        match_head(pre_lon, pre_lat, endlon, endlat, imu_heading)
    elif dist <= 3:
        print("Reached. Distance remaining",dist)
        return
    elif check_clear():
        mask()
    
    rospy.spin()

if __name__ == '__main__':
    main()
