#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def publish():
	pub = rospy.Publisher('radius',Float64,queue_size=10)
	rospy.init_node('radius_pub',anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		radius=1
		rospy.loginfo(radius)
		pub.publish(radius)
		rate.sleep()
if __name__=="__main__":
	try:
		publish()
	except rospy.ROSInterruptException:
		pass