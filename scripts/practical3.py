#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

import random
import rospy
from geometry_msgs.msg import Twist

sideLength = 2
numOfCorners = 4
startTime = 0
endTime = 0

def forward():
    if (0.5 * time) >= 2:
        msg.linear.x = 0
        msg.angular.z = 0.2
        startTime = rospy.get_time()
      
def turn():
	x = 1
	startTime = rospy.get_time()
	while x == 1:
		endTime = rospy.get_time() - startTime
		rospy.loginfo("Time taken: s%", endTime)
		if endTime > 7.85:
			x = 2

def pubvel():
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
	
	rospy.sleep(1)
	# Loop at 2Hz until the node is shutdown
	rate = rospy.Rate(2)
	
	startTime = rospy.get_time()

	while not rospy.is_shutdown():
		twist = Twist()
		twist.linear.x = 0.2
		for i in range(20):
			pub.publish(twist)
			rospy.loginfo("Moving forward")
			rate.sleep()
		twist = Twist()
		twist.angular.z = 0.2
		for i in range(16):
			pub.publish(twist)
			rospy.loginfo("Turning")
			rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
