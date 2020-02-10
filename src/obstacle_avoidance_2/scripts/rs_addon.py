import rospy
from sensor_msgs.msg import PointCloud2 
rospy.init_node("rs_node", anonymous=True, disable_signals=True)
i=0
def rs_callback(msg):	
	global i
	if len(msg.data) !=0 :
		i+=1
		l=list(bytearray(msg.data))
		if len(l)%3==0:
			print(l)
		#print()
def main():

	rospy.Subscriber("/obstacle", PointCloud2,rs_callback)
	rospy.spin()
if __name__ == '__main__':
	main()