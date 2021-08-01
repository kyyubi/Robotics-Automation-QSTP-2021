#!/usr/bin/env python
import rospy
from hello_world_pkg.srv import *
from std_msgs.msg import Float64

def callback(data):
	rospy.wait_for_service('compute_ang_vel')
	try:
		ang_vel = rospy.ServiceProxy('compute_ang_vel',turtlebot_ang_vel)
		resp= ang_vel(data.data)
		rospy.loginfo(resp.ang_velocity)
		cmd_vel_pub.publish(resp.ang_velocity)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

if __name__=="__main__":
	cmd_vel_pub = rospy.Publisher('cmd_vel',Float64,queue_size=10)
	rospy.init_node('radius_sub',anonymous=True)
	rospy.Subscriber("radius",Float64,callback)
	rospy.spin()