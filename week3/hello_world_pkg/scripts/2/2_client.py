#!/usr/bin/env python
"""Week I Assignment
Simulate the trajectory of a robot approximated using a unicycle model given the
following start states, dt, velocity commands and timesteps
State = (x, y, theta);
Velocity = (v, w) 
1. Start=(0, 0, 0); dt=0.1; vel=(1, 0.5); timesteps: 25
2. Start=(0, 0, 1.57); dt=0.2; vel=(0.5, 1); timesteps: 10
3. Start(0, 0, 0.77); dt=0.05; vel=(5, 4); timestep: 50
Upload the completed python file and the figures of the three sub parts in classroom
"""
import numpy as np
import matplotlib.pyplot as plt
import rospy
from hello_world_pkg.srv import *
from hello_world_pkg.msg import state,control

class unicycle_model:
    def __init__(self, x: float, y: float, theta: float, dt: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.dt = dt
    
        # Store the points of the trajectory to plot
        self.x_points = [self.x]
        self.y_points = [self.y]
    
    def step(self, v: float, w: float, n: int,state1,control1):
        state1.x=self.x
        state1.y=self.y
        state1.theta=self.theta
        control1.v=v
        control1.w=w
        for i in range(n):
        	rospy.wait_for_service('robot_state')
        	try:
        		robot_state = rospy.ServiceProxy('robot_state', robot_server)
        		resp= robot_state(state1, control1)
        		state1.x += resp.updated_state.x*self.dt
        		state1.y += resp.updated_state.y*self.dt
        		state1.theta += resp.updated_state.theta*self.dt
        		self.x_points.append(state1.x)
        		self.y_points.append(state1.y)
        	except rospy.ServiceException as e:
        		print("Service call failed: %s"%e)
        return state1.x, state1.y, state1.theta
    
    def plot(self, v: float, w: float):
        """Function that plots the intermeditate trajectory of the Robot"""
        plt.title(f"Unicycle Model: {v}, {w}")
        plt.xlabel("X-Coordinates")
        plt.ylabel("Y-Coordinates")
        plt.plot(self.x_points, self.y_points, color="red", alpha=0.75)
        plt.grid()
        #plt.show()
        plt.savefig(f"Unicycle_{v}_{w}.png")
    
if __name__ == "__main__":
    x=0
    y=0
    theta=0
    dt=0.1
    v=1
    w=0.5
    timesteps = 25
    state1=state()
    control1=control()
    model = unicycle_model(x,y,theta,dt)
    x,y,theta = model.step(v,w,timesteps,state1,control1)
    model.plot(v,w)
    print("Unicycle Model Assignment")