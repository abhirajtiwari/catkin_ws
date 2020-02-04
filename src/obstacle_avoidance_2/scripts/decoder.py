#!/usr/bin/python3
import serial
from std_msgs.msg import String
import rospy
ser = serial.Serial('/dev/ttyTHS2', 115200)
gear=1
x_joy=y_joy=8000

rospy.init_node("joy_decoder", anonymous=True, disable_signals=True)
def get_values(msg):
    global gear,x_joy,y_joy
    print(msg.data)
    x_joy1,y_joy1,gear1 = msg.data.split(' ')
    x_joy=int(x_joy1)
    y_joy=int(y_joy1)
    gear=int(gear1)
    #print(type(gear))
def joystick_decoder():
    
    rospy.Subscriber("auto_trav_cmd", String, get_values)
    global gear,x_joy,y_joy
    while(True):

        # hill_assist=False
        # gear_pack = 0b00000001
        # if not hill_assist:

        #     gear_pack = (0b00001111 & gear)
        # elif hill_assist:
        #     gear_pack = (0b00001111 & gear)
        #     gear_pack |= 0b00010000

        # x1 = 0b00001111 & (x_joy >> 10)
        # x1 |= 0b00100000

        # x2 = 0b000011111 & (x_joy >> 5)
        # x2 |= 0b01000000

        # x3 = 0b00000000011111 & (x_joy >> 0)
        # x3 |= 0b01100000

        # y1 = 0b00001111 & (y_joy >> 10)
        # y1 |= 0b10000000

        # y2 = 0b000011111 & (y_joy >> 5)
        # y2 |= 0b10100000

        # y3 = 0b00000000011111 & (y_joy >> 0)
        # y3 |= 0b11000000

        ser.write('m'.encode())
        ser.write(gear)
        ser.write('s'.encode())
        ser.write(str(x_joy).encode().zfill(5))
        ser.write('f'.encode())
        ser.write(str(y_joy).encode().zfill(5))
        ser.write('n'.encode())
            
if __name__ == '__main__':
    joystick_decoder()

