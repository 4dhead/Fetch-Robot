#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

import random
import rospy
from geometry_msgs.msg import Twist


def pubvel():
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
	
	# Loop at 2Hz until the node is shutdown
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		# Create and fill in the message.  The other four
		# fields, which are ignored by turtlesim, default to 0.
		msg = Twist()
		msg.linear.x = random.random() #random float 0.0 <= x < 1.0
		msg.angular.z = 2 * random.random() - 1
		
		# Publish the message
		pub.publish(msg)
		
		# Send a message to rosout with the details
		rospy.loginfo("Sending random velocity command: linear=%s angular=%s", msg.linear.x, msg.angular.z)
		
		# Wait until it's time for another iteration
		rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
