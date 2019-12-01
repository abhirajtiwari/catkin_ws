import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geopy import distance
import pyproj
global st_gear, turn_gear
st_gear=5
turn_gear=7
joystick_topic = rospy.Publisher('/joystick_topic', String, queue_size=10)
#Basic Map function
def map1(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Circular to Square Co-ordinates
def ellipticalDiscToSquare(u,v):
    global st_gear, turn_gear
    u = map1 (u, -16, 16, -1, 1)
    v = map1 (v, -16, 16, -1, 1)
    u2 = u * u
    v2 = v * v
    twosqrt2 = 2.0 * (2.0)**(0.5)
    subtermx = 2.0 + u2 - v2
    subtermy = 2.0 - u2 + v2
    termx = subtermx + u * twosqrt2
    termx2 = subtermx - u * twosqrt2
    termy = subtermy + v * twosqrt2
    termy2 = subtermy - v * twosqrt2
    x = 0.5 * (termx)**(0.5) - 0.5 * (termx2)**(0.5)
    y = 0.5 * (termy)**(0.5) - 0.5 * (termy2)**(0.5)
    x = map1(x, -1, 1, -8000, 8000) + 8000
    y = 8000 + map1(y, -1, 1, -8000, 8000)
    publish_joystick(st_gear, x, y)

def publish_joystick(gear, x, y):
    joystick_topic.publish("{} {} {}".format(gear,x, y)) 

def anticlockwise():
    global st_gear, turn_gear
    publish_joystick(turn_gear, 0, 8000)
    print('Rotating anticlockwise')

def clockwise():
    global st_gear, turn_gear
    publish_joystick(turn_gear, 16000, 8000)
    print('Rotating clockwise')
    

def get_heading(startlong, startlat, endlong, endlat):
    (az12, az21, dist) = g.inv(startlong, startlat, endlong, endlat)
    if az12<0:
       az12=az12+360
    return az12,dist 

def match_head():
    waypoint_heading,dist=get_heading(startlong, startlat, endlong, endlat)
    while True:
        imu_heading=int(heading)
        heading_diff=imu_heading-waypoint_heading
        print("Heading",imu_heading,"Bearing",waypoint_heading,"Difference",heading_diff)
        if imu_heading < waypoint_heading+5 and imu_heading>waypoint_heading-5:
            #brute_stop()
            break
        if heading_diff<=0 and heading_diff >= -180:
            clockwise()
        if heading_diff < -180:
            anticlockwise()
        if heading_diff>=0 and heading_diff < 180:
            anticlockwise()             
        if heading_diff >= 180:
            clockwise()
