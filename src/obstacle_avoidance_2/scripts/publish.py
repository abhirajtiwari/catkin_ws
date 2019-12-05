import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geopy import distance
import pyproj
import serial
global st_gear, turn_gear
import math
st_gear=7
turn_gear=10
pub_j = rospy.Publisher('joystick_encoder', String, queue_size=10)
g = pyproj.Geod(ellps='WGS84')
#Basic Map function
def map1(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Circular to Square Co-ordinates
def ellipticalDiscToSquare(u,v):
    global st_gear, turn_gear
    '''if math.degrees(math.atan2(v,u))>90:
        anticlockwise()
        return
    elif math.degrees(math.atan2(v,u))< -90:
        clockwise()
        return'''
    rang = ((u**2)+(v**2))**0.5
    u = map1 (u, -rang, rang, -1, 1)
    v = map1 (v, -rang, rang, -1, 1)
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
    print("Josytick",st_gear,x,y)
    rospy.logdebug("Joystickx-"+str(x)+ "y-"+str(y))
    joystick_decoder(int(x),int(y),st_gear,False)



def anticlockwise():
    global st_gear, turn_gear
    joystick_decoder(0, 8000, turn_gear, False)
    print('Rotating anticlockwise')

def clockwise():
    global st_gear, turn_gear
    joystick_decoder(16000, 8000, turn_gear, False)
    print('Rotating clockwise')
def forward():
    global st_gear, turn_gear
    joystick_decoder(8000, 16000, turn_gear, False)
    print('Going Forward')
def brute_stop():
    global st_gear, turn_gear
    joystick_decoder(8000, 8000, turn_gear, False)
    print('Rotating clockwise')

#Heading for destinaion co-ordinates
def get_heading(start, end):
    (az12, az21, dist) = g.inv(start[1], start[0], end[1], end[0])
    if az12<0:
       az12=az12+360
    return az12,dist 

def match_head(start, end, heading_diff):
    global st_gear, turn_gear
    if abs(heading_diff) <= 30 or abs(heading_diff) >= 330:
        turn_gear=4
    elif abs(heading_diff) <= 20 or abs(heading_diff) >= 340:
        turn_gear=2
    elif abs(heading_diff) <= 10 or abs(heading_diff) >= 350:
        turn_gear=2
    else:
        turn_gear=7
    
    if heading_diff < 180:
        anticlockwise()             
    elif heading_diff >= 180:
        clockwise()

ob1  =  String()
def joystick_decoder(x_joy,y_joy,gear,hill_assist):
    global ob1
    ob1.data = "{} {} {}".format(x_joy, y_joy, gear)
    #print("ob1 " + ob1.data)
    pub_j.publish(ob1)
'''
    gear_pack = 0b00000001
    if not hill_assist:

        gear_pack = (0b00001111 & gear)
    elif hill_assist:
        gear_pack = (0b00001111 & gear)
        gear_pack |= 0b00010000

    x1 = 0b00001111 & (x_joy >> 10)
    x1 |= 0b00100000

    x2 = 0b000011111 & (x_joy >> 5)
    x2 |= 0b01000000

    x3 = 0b00000000011111 & (x_joy >> 0)
    x3 |= 0b01100000

    y1 = 0b00001111 & (y_joy >> 10)
    y1 |= 0b10000000

    y2 = 0b000011111 & (y_joy >> 5)
    y2 |= 0b10100000

    y3 = 0b00000000011111 & (y_joy >> 0)
    y3 |= 0b11000000
'''
    
    
    
