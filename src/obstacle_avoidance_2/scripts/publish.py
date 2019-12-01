import rospy
from sensor_msgs.msg import NavSatFix, Imu, String
import tf

def map1(x,in_min,in_max,out_min,out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def ellipticalDiscToSquare(u,v):
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
	joystick_topic = rospy.Publisher('/joystick_topic', String, queue_size=10)
    joystick_topic.publish( str(x) + " " + str(y)) 