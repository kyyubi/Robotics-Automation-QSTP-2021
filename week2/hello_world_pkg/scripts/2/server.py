#!/usr/bin/env python
import rospy
from hello_world_pkg.srv import robot_server,robot_serverResponse
import numpy as np
from hello_world_pkg.msg import state
updated_state=state()
def handle(req):
	print(req)
	updated_state.x = req.model_control.v*np.cos(req.model_state.theta)
	updated_state.y = req.model_control.v*np.sin(req.model_state.theta)
	updated_state.theta = req.model_control.w
	return robot_serverResponse(updated_state)
def robot_server_server():
	rospy.init_node('robot_state_server')
	s = rospy.Service('robot_state',robot_server,handle)
	print("Ready")
	rospy.spin()

if __name__ == "__main__":
	robot_server_server()