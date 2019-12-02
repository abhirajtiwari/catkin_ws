import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geopy import distance
import pyproj
global st_gear, turn_gear
st_gear=5
turn_gear=7
joystick_topic = rospy.Publisher('/joystick_topic', String, queue_size=10)
g = pyproj.Geod(ellps='WGS84')

#Basic Map function
def map1(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Circular to Square Co-ordinates
def ellipticalDiscToSquare(u,v):
    print(11111)
    global st_gear, turn_gear
    u = map1 (u, -160, 160, -1, 1)
    v = map1 (v, -160, 160, -1, 1)
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
    print(x,y)
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

def brute_stop():
    global st_gear, turn_gear
    publish_joystick(st_gear, 8000, 8000)
    print('Rotating clockwise')

#Heading for destinaion co-ordinates
def get_heading(start, end):
    (az12, az21, dist) = g.inv(start[1], start[0], end[1], end[0])
    if az12<0:
       az12=az12+360
    return az12,dist 

def match_head(start, end, heading_diff):
    global st_gear, turn_gear
    if abs(heading_diff) <= 20:
        turn_gear=5
    elif abs(heading_diff) <= 10:
        turn_gear=4
    if heading_diff <= 0 and heading_diff >= -180:
        clockwise()
    elif heading_diff < -180:
        anticlockwise()
    elif heading_diff >= 0 and heading_diff < 180:
        anticlockwise()             
    elif heading_diff >= 180:
        clockwise()
