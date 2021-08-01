#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import String
import numpy as np
import math
from geometry_msgs.msg import Twist
import tf

class path_controller():
	def __init__(self,path):
		self.kp_dist = 0.2
		self.kd_dist = 0.0
		self.ki_dist = 0
		self.kp_ang = 0.2
		self.kd_ang = 0
		self.ki_ang = 0
		self.path=path
		self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.rate = rospy.Rate(10)
		self.velocity=Twist()
		self.velocity.linear.x=0
		self.velocity.angular.z=0
		self.points = []
		self.x_pose=0
		self.y_pose=0
		self.yaw=0

	def position_callback(self,data):
		self.x_pose = data.pose.pose.position.x
		self.y_pose = data.pose.pose.position.y
		quaternion = (
    	data.pose.pose.orientation.x,
    	data.pose.pose.orientation.y,
    	data.pose.pose.orientation.z,
    	data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.yaw = euler[2]

	def path_callback(self,data):
		self.points=[]
		for i in data.poses:
			self.points.append([i.pose.position.x,i.pose.position.y])


	def path_calculation(self):
		sub=rospy.Subscriber('/odom',Odometry,self.position_callback)
		topic='/path'+str(self.path)
		path_sub = rospy.Subscriber(topic,Path,self.path_callback)
		rospy.sleep(1)
		i=0
		prev_error_dist=0
		prev_error_ang=0
		total_error_dist=0
		total_error_ang=0
		while not rospy.is_shutdown():
			x_goal = self.points[i][0]
			y_goal = self.points[i][1]
			error_distance = np.sqrt((self.x_pose-x_goal)**2+(self.y_pose-y_goal)**2)
			if error_distance>0.1:
				self.ki_dist=0
				error_ang = math.atan2(y_goal-self.y_pose,x_goal-self.x_pose)
				if y_goal-self.y_pose >= 0 and x_goal-self.x_pose >= 0:
					error_ang = abs(error_ang) - self.yaw
				elif y_goal-self.y_pose <= 0 and x_goal-self.x_pose >= 0:
					error_ang = -abs(error_ang) - self.yaw
				elif y_goal-self.y_pose >= 0 and x_goal-self.x_pose < 0:
					if self.yaw < -0.1:
						self.yaw += 2 * math.pi
						error_ang = math.pi - abs(error_ang) - self.yaw
						self.yaw = self.yaw - 2*math.pi
					else:
						error_ang = math.pi - abs(error_ang) - self.yaw
				elif y_goal-self.y_pose <= 0 and x_goal-self.x_pose <= 0:
					if self.yaw < -0.1:
						self.yaw += 2 * math.pi
						error_ang = math.pi + abs(error_ang) - self.yaw
						self.yaw = self.yaw - 2*math.pi
					else:
						error_ang = math.pi + abs(error_ang) - self.yaw
				error_distance_diff = error_distance - prev_error_dist
				total_error_dist += error_distance
				error_ang_diff = error_ang - prev_error_ang
				total_error_ang += error_ang
				linear_vel = self.kp_dist*error_distance + self.kd_dist*error_distance_diff + self.ki_dist*total_error_dist
				ang_vel = self.kp_ang*error_ang + self.kd_ang*error_ang_diff + self.ki_ang*total_error_ang
				
				if linear_vel > 0.5:
					linear_vel = 0.5
				elif linear_vel < (-0.5):
					linear_vel = (-0.5)
				if ang_vel > 0.5:
					ang_vel = 0.5
				elif ang_vel < (-0.5):
					ang_vel = (-0.5)
				self.velocity.linear.x = linear_vel
				self.velocity.angular.z = ang_vel
				
				self.pub.publish(self.velocity)
				prev_error_dist = error_distance 
				prev_error_ang = error_ang
				self.rate.sleep()
			elif(len(self.points)-1)==i:
				print("Reached")
				self.velocity.linear.x = 0
				self.velocity.angular.z = 0
				self.pub.publish(self.velocity)
			else:
				print("here")
				error_ang = math.atan2(self.points[i+1][1] - self.y_pose,self.points[i+1][0] - self.x_pose)
				error_ang = error_ang - self.yaw
				if self.points[i+1][1]-self.y_pose >= 0 and self.points[i+1][0]-self.x_pose >= 0:
					error_ang = abs(error_ang) - self.yaw
				elif self.points[i+1][1]-self.y_pose <= 0 and self.points[i+1][0]-self.x_pose >= 0:
					error_ang = -abs(error_ang) - self.yaw
				elif self.points[i+1][1]-self.y_pose >= 0 and self.points[i+1][0]-self.x_pose <= 0:
					if self.yaw < -0.1:
						self.yaw += 2 * math.pi
						error_ang = math.pi - abs(error_ang) - self.yaw
						self.yaw = self.yaw - 2*math.pi
					else:
						error_ang = math.pi - abs(error_ang) - self.yaw
				elif (self.points[i+1][1]-self.y_pose <= 0) and (self.points[i+1][0]-self.x_pose <= 0):
					if self.yaw < -0.1:
						self.yaw += 2 * math.pi
						error_ang = math.pi + abs(error_ang) - self.yaw
						self.yaw = self.yaw - 2*math.pi
					else:
						error_ang = math.pi + abs(error_ang) - self.yaw
                
				error_ang_diff = error_ang - prev_error_ang
				total_error_ang += error_ang
				ang_vel = self.kp_ang*error_ang + self.kd_ang*error_ang_diff + self.ki_ang*total_error_ang
				self.velocity.linear.x = 0
				if ang_vel > 0.5:
					ang_vel = 0.5
				elif ang_vel < (-0.5):
					ang_vel = (-0.5)
				self.velocity.angular.z = ang_vel
				self.pub.publish(self.velocity)
				if abs(error_ang) < 0.05:
					self.ki_dist = 0.1
					i+=1
			self.rate.sleep()

if __name__=="__main__":
	rospy.init_node("path")
	path=3
	controller = path_controller(path)
	controller.path_calculation()
	rospy.spin()