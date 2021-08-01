#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Twist
from tf.transformations import euler_from_quaternion
import sys

class ControlCalculator:
    def __init__(self, path):
        self.path = path
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.velocity = Twist()
        self.path_nodes = [[0, 0]]
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.k = 0
    def pose_callback(self, msg):
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (roll, pitch, self.yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.yaw_deg = math.degrees(self.yaw)


    def path_callback(self, msg):
        self.path_nodes = [[0,0]]
        for i in range(0, len(msg.poses)):
            self.path_nodes.append([msg.poses[i].pose.position.x, msg.poses[i].pose.position.y])
        

    def get_path(self):
        if self.path == '1':
            path_sub_1 = rospy.Subscriber('/path1', Path, self.path_callback)
            rospy.sleep(1)
        elif self.path == '2':
            path_sub_2 = rospy.Subscriber('/path2', Path, self.path_callback)
            rospy.sleep(1)
        elif self.path == '3':
            path_sub_3 = rospy.Subscriber('/path3', Path, self.path_callback)
            rospy.sleep(1)

    def calculations(self):
        odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.get_path()
        count = 1
        pid_dist = pid_controller(0.2, 0, 0, 0.5)
        pid_yaw = pid_controller(0.2, 0, 0, 0.5)
        while not rospy.is_shutdown():
            goal_x = self.path_nodes[count][0]
            goal_y = self.path_nodes[count][1]
            distance = math.sqrt(math.pow((goal_x - self.x_pose), 2) + math.pow((goal_y - self.y_pose), 2))


            if distance >= 0.1:
                psi = math.atan((goal_y - self.y_pose)/(goal_x - self.x_pose))
                
                if goal_y-self.y_pose >= 0 and goal_x-self.x_pose >= 0:
                    error = abs(psi) - self.yaw
                elif goal_y-self.y_pose <= 0 and goal_x-self.x_pose >= 0:
                    error = -abs(psi) - self.yaw
                elif goal_y-self.y_pose >= 0 and goal_x-self.x_pose < 0:
                    if self.yaw < -0.1:
                        self.yaw += 2 * math.pi
                        error = math.pi - abs(psi) - self.yaw
                        self.yaw = self.yaw - 2*math.pi
                    else:
                        error = math.pi - abs(psi) - self.yaw
                elif goal_y-self.y_pose <= 0 and goal_x-self.x_pose <= 0:
                    if self.yaw < -0.1:
                        self.yaw += 2 * math.pi
                        error = math.pi + abs(psi) - self.yaw
                        self.yaw = self.yaw - 2*math.pi
                    else:
                        error = math.pi + abs(psi) - self.yaw

                
                out_yaw = pid_yaw.set_current_error(error)
                out_distance = pid_dist.set_current_error(distance)
                self.action(out_distance, out_yaw)

            else:
                if count == len(self.path_nodes) - 1:
                    print("Goal Reached!")
                    self.velocity.linear.x = 0
                    self.velocity.angular.z = 0
                    self.pub.publish(self.velocity)
                    break
                if self.k == 0:
                    pid_yaw = pid_controller(0.2, 0, 0, 0.5)
                    self.k = 1
                psi = math.atan((self.path_nodes[count+1][1] - self.y_pose)/(self.path_nodes[count+1][0] - self.x_pose))
                error = psi - self.yaw
                if self.path_nodes[count+1][1]-self.y_pose >= 0 and self.path_nodes[count+1][0]-self.x_pose >= 0:
                    error = abs(psi) - self.yaw
                elif self.path_nodes[count+1][1]-self.y_pose <= 0 and self.path_nodes[count+1][0]-self.x_pose >= 0:
                    error = -abs(psi) - self.yaw
                elif self.path_nodes[count+1][1]-self.y_pose >= 0 and self.path_nodes[count+1][0]-self.x_pose <= 0:
                    if self.yaw < -0.1:
                        self.yaw += 2 * math.pi
                        error = math.pi - abs(psi) - self.yaw
                        self.yaw = self.yaw - 2*math.pi
                    else:
                        error = math.pi - abs(psi) - self.yaw

                elif self.path_nodes[count+1][1]-self.y_pose <= 0 and self.path_nodes[count+1][0]-self.x_pose <= 0:
                    if self.yaw < -0.1:
                        self.yaw += 2 * math.pi
                        error = math.pi + abs(psi) - self.yaw
                        self.yaw = self.yaw - 2*math.pi
                    else:
                        error = math.pi + abs(psi) - self.yaw
                out_yaw = pid_yaw.set_current_error(error)   
                self.action(0, out_yaw)
                if abs(error) < 0.05:
                    pid_dist = pid_controller(0.2, 0, 0.1, 0.5)
                    pid_yaw = pid_controller(0.2, 0, 0, 0.5)
                    self.k = 0
                    count += 1

            self.rate.sleep()

    def action(self, d, y):
        self.velocity.linear.x = d
        self.velocity.angular.z = y
        self.pub.publish(self.velocity)

class pid_controller:
    def __init__(self, p, i, d, lim):
        self.kp = p
        self.ki = i
        self.kd = d
        self.lim = lim
        self.prev_error = 0.0
        self.error_int = 0

    def set_current_error(self, error):
        output_p = error * self.kp

        error_diff = error - self.prev_error
        output_d = error_diff * self.kp

        self.error_int = self.error_int + self.prev_error
        output_i = self.ki * self.error_int

        self.prev_error = error
        output = output_p + output_i + output_d

        if output > self.lim:
            output = self.lim
        elif output < (-self.lim):
            output = (-self.lim)

        return output

if __name__ == '__main__':
    rospy.init_node('PID_Controller')
    #if len(sys.argv) == 2:
    #    path = int(sys.argv[1])
    path = '1'
    a = ControlCalculator(path)
    a.calculations()
    rospy.spin()
