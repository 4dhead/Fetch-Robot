#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw
import rospy
import math

from sensor_msgs.msg import LaserScan

# A callback function. Executed each time a new pose message arrives
def LaserScanMessageReceived(msg):
    if math.isinf(msg.ranges[25]):
	    rospy.loginfo("%.2f",msg.ranges[0])

if __name__ == "__main__":
	# Initialise the ROS system and become a node
	rospy.init_node('subscribe_to_LaserScan', anonymous=True)

	# Create a subscriber object
	rospy.Subscriber('velodyne/scan', LaserScan, LaserScanMessageReceived)
	
	# Let ROS take over
	rospy.spin()
	
