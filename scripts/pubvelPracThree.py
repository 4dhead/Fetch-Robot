#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

import random
import rospy
from geometry_msgs.msg import Twist
from bed45_ros360.srv import Shape, ShapeResponse

sideLengthParam = "sideLength"
numOfCornersParam = "numOfCorners"
sideLength = 2
numOfCorners = 4

startTime = 0
endTime = 0
state = 1
turningSpeed = 0.2
movingSpeed = 0.5

def setShape_cb(request):
	#service behavours...
	sideLength = request.sideLength
	numOfCorners = request.numOfCorners
	#TODO add behaviours, update param server
	rospy.set_param(sideLengthParam, sideLength)
	rospy.set_param(numOfCornersParam, numOfCorners)
	
	#Define response (calculate perimeter length)
	response = request.sideLength * request.numOfCorners
	
	#returning will send the response back to the client
	return ShapeResponse(response)

def paramCheck(paramToBeChecked):
	if rospy.has_param(paramToBeChecked):
		myvar = rospy.get_param(paramToBeChecked)
		rospy.loginfo("Got param %s: %s", paramToBeChecked, myvar)
		
		return myvar
	else:
		rospy.logerr("%s param did not exist", paramToBeChecked)

def pubvel():
	global numOfCorners
	global sideLength
	
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
	
	# Service Advert
	s = rospy.Service('SetShapeService', Shape, setShape_cb)
	rospy.loginfo("Ready to serve.")
	
	# Loop at 2Hz until the node is shutdown
	rate = rospy.Rate(10)
	
	# Turn angle radians
	angleInRadians = (360/numOfCorners)/180 * 3.14
	
	#--QUESTION--refrenced before stated if i dont put this here, cant see global?
	state = 1
	
	startTime = rospy.get_time()
	while not rospy.is_shutdown():
        # Timer
		endTime = rospy.get_time() - startTime
		
		# Set Angle
		angleInRadians = (360/numOfCorners)/180 * 3.14
		
		# Create Message
		msg = Twist()
		
		if state == 1:
			msg.angular.z = 0.0
			msg.linear.x = movingSpeed
			pub.publish(msg)
			rospy.loginfo("linear=%s angular=%s", msg.linear.x, msg.angular.z)
			rospy.loginfo("Time taken: %s", endTime)
		
		if (movingSpeed * endTime) >= sideLength and state == 1:
			state = 2
			msg.linear.x = 0
			#msg.angular.z = turningSpeed
			#--QUESTION--do we have to publish after we update?
			pub.publish(msg)
			rospy.loginfo("--Stopping--")
			startTime = rospy.get_time()
			
		if state == 2:
			sideLength = paramCheck(sideLengthParam)
			numOfCorners = paramCheck(numOfCornersParam)
			#--QUESTION--how to update from cmd line???
			rospy.loginfo("--Turning--")
			endTime = rospy.get_time() - startTime
			rospy.loginfo("Time taken turning: %s StartTime: %s", endTime, startTime)
			msg.linear.x = 0
			msg.angular.z = turningSpeed
			pub.publish(msg)
			
		if (turningSpeed * endTime) >= angleInRadians and state == 2:
			state = 1
			startTime = rospy.get_time()

		#while state == 2:
		#	rospy.loginfo("--Turning--")
		#	endTime = rospy.get_time() - startTime
		#	rospy.loginfo("Time taken turning: %s StartTime: %s", endTime, startTime)
		#	msg.angular.z = 0.2
		#	pub.publish(msg)
		#	if (0.2 * endTime) >= angleInRadians:
		#		#msg.angular.z = 0.0
		#		state = 1
		#		startTime = rospy.get_time()
				
			#--QUESTION--Do i need this inside this loop?
		#	rate.sleep()
		
		# Wait until it's time for another iteration
		rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
