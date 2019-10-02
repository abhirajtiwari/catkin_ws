#!/usr/bin/env python

#Import ROS stuff
import rospy 
from sensor_msgs.msg import Imu, MagneticField

#Import socket for getting data from android
import socket


#Server setup for recieving GPS data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 5555
server_address = ('', port)
sock.bind(server_address)

#ROS setup
rospy.init_node('imu_publisher.py', anonymous=True)
pub_i = rospy.Publisher('/phone/imu/data_raw', Imu, queue_size=10)
pub_m = rospy.Publisher('/phone/imu/mag', MagneticField, queue_size=10)
imu_data = Imu()
imu_data.header.frame_id = 'imu'
imu_data.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
mag_data = MagneticField()
mag_data.header.frame_id = 'imu'

#Data collection
ax = ay = az = gx = gy = gz = mx = my = mz = 0.0
try:
    while True:
        data = sock.recvfrom(10000000)
        data = map(float, data[0].split(','))
        print data
        l = len(data)
        time = rospy.Time.now()

        #accel
        if data[-4] == 82:
            ax = data[-3]
            ay = data[-2]
            az = data[-1]

        #gyro and mag
        if l>5 and data[5] == 4: #SHORT-CIRCUITING
            gx = data[6]
            gy = data[7]
            gz = data[8]
            
            if l>9:
                mx = data[10]
                my = data[11]
                mz = data[12]
        elif l>5 and data[5] == 5:
            mx = data[6]
            my = data[7]
            mz = data[8]

            if l>9:
                gx = data[10]
                gy = data[11]
                gz = data[12]

        #Publish
        imu_data.header.stamp = time
        mag_data.header.stamp = time

        imu_data.angular_velocity.x = gx
        imu_data.angular_velocity.y = gy
        imu_data.angular_velocity.z = gz

        imu_data.linear_acceleration.x = ax
        imu_data.linear_acceleration.y = ay
        imu_data.linear_acceleration.z = az

        mag_data.magnetic_field.x = mx
        mag_data.magnetic_field.y = my
        mag_data.magnetic_field.z = mz

        pub_i.publish(imu_data)
        pub_m.publish(mag_data)

        # print '\n------------\nAccel:\t', ax, ay, az, '\nGyro:\t', gx, gy, gz, '\nMag:\t', mx, my, mz, '\n'
finally:
    sock.close()

