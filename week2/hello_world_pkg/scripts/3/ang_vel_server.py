#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
linear_vel=0.1
from hello_world_pkg.srv import turtlebot_ang_vel,turtlebot_ang_velResponse

def handle(req):
	ang_vel = linear_vel/req.radius
	return turtlebot_ang_velResponse(ang_vel)

def angular_velocity_serv():
	rospy.init_node('angular_velocity_serv')
	s = rospy.Service('compute_ang_vel',turtlebot_ang_vel,handle)
	print("Ready")
	rospy.spin()

if __name__=="__main__":
	angular_velocity_serv()