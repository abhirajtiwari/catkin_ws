#!/usr/bin/env python

#Import ROS stuff
import rospy 
from nmea_msgs.msg import Sentence

#Import socket for getting data from android
import socket


#Server setup for recieving GPS data
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 6000
client_address = ('192.168.43.10', port)

#Init rospy stuff
rospy.init_node("gps_publisher.py", anonymous=True)
pub = rospy.Publisher('/nmea_sentence', Sentence, queue_size=10)
s = Sentence()

try:
    sock.connect(client_address)
    while True:
        data = sock.recv(10000000)
        if data[0] == 'G':
            s.sentence = '$'+data[0:data.find('\r')]
            s.header.stamp = rospy.Time.now()
            s.header.frame_id = 'gps'
            pub.publish(s)
finally:
    sock.close()

