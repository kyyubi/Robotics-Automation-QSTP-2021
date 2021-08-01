#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def hello_pub():
	pub = rospy.Publisher('hello',String,queue_size=10)
	rospy.init_node('hello_pub',anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = "Hello,"
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
if __name__ == '__main__':
	try:
		hello_pub()
	except rospy.ROSInterruptException:
		pass