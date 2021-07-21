#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def hello_callback(data,pub_data):
	pub_data += data.data
	rospy.Subscriber("world",String,world_callback,pub_data)

def world_callback(data,pub_data):
	pub_data += data.data
	#print(pub_data)
	pub.publish(pub_data)
	rospy.loginfo(pub_data)


if __name__ == '__main__':
	pub_data=""
	pub = rospy.Publisher("helloworld",String,queue_size=10)
	rospy.init_node('hello_world',anonymous=True)
	rospy.Subscriber("hello",String,hello_callback,pub_data)
	rospy.spin()
