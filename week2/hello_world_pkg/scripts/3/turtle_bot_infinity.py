#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
global flag,flag_count,radius
def callback(data):
	global flag,flag_count
	if (np.abs(data.pose.pose.position.x)<0.01)&(np.abs(data.pose.pose.position.y)<0.01):
		if (flag==0)&(flag_count>100):
			flag=1
			flag_count=0
		elif flag_count>100:
			flag=0
			flag_count=0
def radius_callback(data):
	global radius
	radius=data.data

def publisher():
	global flag,flag_count,radius
	pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
	rospy.init_node("turtle_bot_publisher",anonymous=True)
	rate=rospy.Rate(10)
	twist = Twist()
	flag=0
	flag_count=0
	twist.linear.x=0.1
	radius=0.9
	while not rospy.is_shutdown():
		rospy.Subscriber("radius",Float64,radius_callback)
		if flag==1:
			twist.angular.z=-twist.linear.x/radius
		else:
			twist.angular.z=twist.linear.x/radius
		pub.publish(twist)
		flag_count+=1
		rospy.Subscriber("/odom",Odometry,callback)
		rate.sleep()
if __name__=="__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
