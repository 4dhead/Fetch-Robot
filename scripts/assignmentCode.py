#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

import random
import rospy
import tf2_ros
import tf_conversions
import cv2
import actionlib
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from opencv_apps.msg import MomentArrayStamped
from bed45_ros360.srv import Shape, ShapeResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan

bridge = CvBridge()

startTime = 0
endTime = 0
state = "find_ball"

#location of object on screen
ballLocationX = 0
ballLocationY = 0

#location of bin on screen
binLocationX = 0
binLocationY = 0

#offset for distance from camera to object
offsetX = 0
offsetY = 0

#depth variables for both object and bin
depth = 0
depthBin = 0

#laser variables for avoiding objects set greater then avoidance threshhold
#to avoid any strange behaviour
laserFront = 10 #greater then threshold to dodge object
laserLeft = 10
laserRight = 10

#function for calculating offset for distance camera to object
def offset(depth, x, y):
	f = 554 #focal length constant
	w = 640 #width of screen
	h = 480 #height of screen
	
	offsetX = -((x - (w/2))/f)*depth
	
	offsetY = -((y - (h/2))/f)*depth
	
	return offsetX, offsetY

#function for detrming distance to object from camera. 
def cb_depthImage(image):
	global bridge, x, y, depth
	x = int(ballLocationX)
	y = int(ballLocationY)
	rospy.loginfo('ball x %s',x)
	rospy.loginfo('ball y %s',y)
	# image msg obtained from callback message for topic
	try:
			cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
			# where x,y is the centre point from the published moment
			depth = cv_image[y][x]
			rospy.loginfo('Depth of point is %s m',depth)
			# For testing/verification:
			cv2.circle(cv_image, (x,y), 10, 255) # draw circle radius 10 at x,y
			cv2.imshow("Image window", cv_image) # display the image
			cv2.waitKey(3)
			
			if not math.isnan(depth):
			
				br = tf2_ros.TransformBroadcaster()
				t = TransformStamped()
				
				offsetX, offsetY = offset(depth, x, y)

				t.header.stamp = rospy.Time.now()
				t.header.frame_id = "head_camera_depth_frame"
				t.child_frame_id = "target_object"
				t.transform.translation.x = depth
				t.transform.translation.y = offsetX #x_offset 
				t.transform.translation.z = offsetY #y_offset

				q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0) # no rotation
				t.transform.rotation.x = q[0]
				t.transform.rotation.y = q[1]
				t.transform.rotation.z = q[2]
				t.transform.rotation.w = q[3]
				br.sendTransform(t)
	except CvBridgeError as e:
		print(e)

#function for detrming distance to bin from camera
def cb_depthImageBin(image):
	global bridge, x, y, depthBin
	x = int(binLocationX)
	y = int(binLocationY)
	rospy.loginfo('bin x %s',x)
	rospy.loginfo('bin y %s',y)
	# image msg obtained from callback message for topic
	try:
			cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
			# where x,y is the centre point from the published moment
			depthBin = cv_image[y][x]
			rospy.loginfo('DepthBin of point is %s m',depthBin)
			# For testing/verification:
			cv2.circle(cv_image, (x,y), 10, 255) # draw circle radius 10 at x,y
			cv2.imshow("Image window", cv_image) # display the image
			cv2.waitKey(3)
			
			if not math.isnan(depthBin):
			
				br = tf2_ros.TransformBroadcaster()
				t = TransformStamped()
				
				offsetX, offsetY = offset(depthBin, x, y)

				t.header.stamp = rospy.Time.now()
				t.header.frame_id = "head_camera_depthBin_frame"
				t.child_frame_id = "target_object"
				t.transform.translation.x = depthBin
				t.transform.translation.y = offsetX #x_offset 
				t.transform.translation.z = offsetY #y_offset

				q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0) # no rotation
				t.transform.rotation.x = q[0]
				t.transform.rotation.y = q[1]
				t.transform.rotation.z = q[2]
				t.transform.rotation.w = q[3]
				br.sendTransform(t)
	except CvBridgeError as e:
		print(e)

#function for setting x/y coords of blue objects on screen
def MomentArrayStampedMessageRecievedBlue(msg):
	global ballLocationX
	global ballLocationY
	
	try:
		rospy.loginfo("Blue Found At :X: %.2f, Y: %.2f",msg.moments[0].center.x,msg.moments[0].center.y)
		ballLocationX = msg.moments[0].center.x
		ballLocationY = msg.moments[0].center.y
	except Exception as e:
		rospy.loginfo("No Blue Detected")

#function for setting x/y coords of red objects on screen
def MomentArrayStampedMessageRecievedRed(msg):
	global binLocationX
	global binLocationY
	
	try:
		rospy.loginfo("Red Found At :X: %.2f, Y: %.2f",msg.moments[0].center.x,msg.moments[0].center.y)
		binLocationX = msg.moments[0].center.x
		binLocationY = msg.moments[0].center.y
	except Exception as e:
		rospy.loginfo("No Red Detected")
		rospy.loginfo("binLocationX: %s, binLocationY: %s", binLocationX, binLocationY)

#function for locating objects infront of front, left, right lasers
#def LaserScanMessageReceived(msg):
#	global laserFront
#	global laserLeft
#	global laserRight
#	rospy.loginfo("%s", msg.ranges[331])
#	laserFront = msg.ranges[331] #center laser
#	laserLeft = msg.ranges[60] #leftLaser
#	laserRight = msg.ranges[600] #rightLaser

#this function makes use of all previous ones and moves the robot toward blue objects
#then returns to the bin and repeats. Further work could be done to implement
#arm transformations and object avoidance. Currently robot will become
#confused and not move to items correctly. Hence parts of code for
#object avoidance/arm transfomrations have been removed/commented out to demonstrate
#ability to move between objects and bin
def detectAndRetrieve():
	global depth
	global depthBin
	global ballLocationX
	global ballLocationY
	global binLocationX
	global binLocationY

	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')

	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

	#subscribe to blue moments, red moments, camera depth for objects and bin
	rospy.Subscriber('/contour_moments_blue/moments', MomentArrayStamped, MomentArrayStampedMessageRecievedBlue)
	rospy.Subscriber('/contour_moments_red/moments', MomentArrayStamped, MomentArrayStampedMessageRecievedRed)
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImage)
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImageBin)
	#rospy.Subscriber('/base_scan', LaserScan, LaserScanMessageReceived)
	
	#construct joint in head
	head_joint_names = ["head_pan_joint", "head_tilt_joint"]
	head_joint_positions = [0.0,0.0]

	rospy.loginfo("Waiting for head_controller...")
	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	rospy.loginfo("...connected.")
	
	trajectory = JointTrajectory()
	trajectory.joint_names = head_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	
	#arm joint setup
	#move_group = MoveGroupInterface("arm_with_torso", "base_link")

	# Define ground plane
	# This creates objects in the planning scene that mimic the ground
	# If these were not in place gripper could hit the ground
	#planning_scene = PlanningSceneInterface("base_link")
	#planning_scene.removeCollisionObject("my_front_ground")
	#planning_scene.removeCollisionObject("my_back_ground")
	#planning_scene.removeCollisionObject("my_right_ground")
	#planning_scene.removeCollisionObject("my_left_ground")
	#planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
	#planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
	#planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
	#planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

	# TF joint names
	#joint_names = ["torso_lift_joint", "shoulder_pan_joint",
	#				"shoulder_lift_joint", "upperarm_roll_joint",
	#				"elbow_flex_joint", "forearm_roll_joint",
	#				"wrist_flex_joint", "wrist_roll_joint"]
	# Lists of joint angles in the same order as in joint_names
	#disco_poses = [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]

	# Loop at 10Hz until the node is shutdown
	rate = rospy.Rate(10)

	#set state to find ball and find blue objects
	state = "find_ball" 
	
	startTime = rospy.get_time()
	while not rospy.is_shutdown():
		# Timer
		endTime = rospy.get_time() - startTime

		# Create Message
		msg = Twist()
		
		#----------Below sets arm to position above robot to    --------
		#----------make picking up objects easier               --------
		#----------Commented out as creating issues with finding--------
		#----------bin/objects                                  --------
		
		#if state == "setup_arm":
		#	move_group.moveToJointPosition(joint_names, disco_poses, wait=False)

			# Since we passed in wait=False above we need to wait here
		#	move_group.get_move_action().wait_for_result()
		#	result = move_group.get_move_action().get_result()

		#	if result:
		#		# Checking the MoveItErrorCode
		#		if result.error_code.val == MoveItErrorCodes.SUCCESS:
		#			state = "find_ball"
		#		else:
		#			# If you get to this point please search for:
		#			# moveit_msgs/MoveItErrorCodes.msg
		#			rospy.logerr("Arm goal in state: %s",
		#						 move_group.get_move_action().get_state())
		#	else:
		#		rospy.logerr("MoveIt! failure no result returned.")
		#move_group.get_move_action().cancel_all_goals()

		#state set to search room by rotating till blue object found
		if state == "find_ball":
			msg.angular.z = 0.4
			msg.linear.x = 0.0
			pub.publish(msg)
			rospy.loginfo("linear=%s angular=%s", msg.linear.x, msg.angular.z)
			#rospy.loginfo("Time taken: %s", endTime)
			if ballLocationX > 320: #object detected in center of screen
				state = "move_to_ball"
				msg.angular.z = 0
				pub.publish(msg)

		#state set to move toward blue object when it has been detected
		elif state == "move_to_ball": #object detected
			msg.linear.x = 0.2
			pub.publish(msg)
			rospy.loginfo("BLUE FOUND linear=%s angular=%s", msg.linear.x, msg.angular.z)
			
			#sets position for carmea if it is too low on screen and recenters
			if ballLocationY > 260:
				head_joint_positions[1] += 0.1
				trajectory.points[0].positions = head_joint_positions
				trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
				trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
				trajectory.points[0].time_from_start = rospy.Duration(5.0)

				head_goal = FollowJointTrajectoryGoal()
				head_goal.trajectory = trajectory
				head_goal.goal_time_tolerance = rospy.Duration(0.0)

				rospy.loginfo("Setting positions...")
				head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
				#goalDone_cb, active_cb, feedback_cb) # 

				head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting

			#adjusts robot left or right to center object in screen
			if ballLocationX < 310:
				msg.angular.z = 0.1
				pub.publish(msg)
				rospy.loginfo("BLUE FOUND, altering course to right linear=%s angular=%s", msg.linear.x, msg.angular.z)
			elif ballLocationX > 330:
				msg.angular.z = -0.1
				pub.publish(msg)
				rospy.loginfo("BLUE FOUND, altering course to left linear=%s angular=%s", msg.linear.x, msg.angular.z)
			#if laserFront < 0.5:
			#	state = "avoid"
			#if within 0.65m we have found acceptable distance to grab object
			if depth < 0.65:
				state = "ball_found"

		#state for avoiding objects
		elif state == "avoid":
			if laserFront < 0.5 and laserLeft > 0.5 and laserRight > 0.5:
				msg.linear.x = 0.0
				msg.angular.z = 0.1
				pub.publish(msg)

		#state for picking up object and reseting camrea to neutral for finding bin
		elif state == "ball_found":
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			pub.publish(msg)
			rospy.loginfo("BLUE FOUND, state 3 achieved STOP linear=%s angular=%s", msg.linear.x, msg.angular.z)
			
			head_joint_positions[1] = 0.0
			trajectory.points[0].positions = head_joint_positions
			trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
			trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
			trajectory.points[0].time_from_start = rospy.Duration(5.0)

			head_goal = FollowJointTrajectoryGoal()
			head_goal.trajectory = trajectory
			head_goal.goal_time_tolerance = rospy.Duration(0.0)

			rospy.loginfo("Setting positions...")
			head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
			#goalDone_cb, active_cb, feedback_cb) # 

			head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
			
			depth = 0
			ballLocationX = 0
			ballLocationY = 0
			
			rospy.sleep(3)
			
			state = "find_bin"
		
		#state for fidning bin, similar to find_ball state
		elif state == "find_bin":
			msg.angular.z = 0.4
			msg.linear.x = 0.0
			pub.publish(msg)
			rospy.loginfo("linear=%s angular=%s", msg.linear.x, msg.angular.z)
			#rospy.loginfo("Time taken: %s", endTime)
			if binLocationX > 300 and binLocationX < 340: #object detected in center of screen
				state = "move_to_bin"
				msg.angular.z = 0
				pub.publish(msg)
				
		#state for moving to bin when it has been detected
		elif state == "move_to_bin": #object detected
			msg.linear.x = 0.2
			#pub.publish(msg)
			rospy.loginfo("RED FOUND linear=%s angular=%s", msg.linear.x, msg.angular.z)
			
			if binLocationY > 260:
				head_joint_positions[1] += 0.1
				trajectory.points[0].positions = head_joint_positions
				trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
				trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
				trajectory.points[0].time_from_start = rospy.Duration(5.0)

				head_goal = FollowJointTrajectoryGoal()
				head_goal.trajectory = trajectory
				head_goal.goal_time_tolerance = rospy.Duration(0.0)

				rospy.loginfo("Setting positions...")
				head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
				#goalDone_cb, active_cb, feedback_cb) # 

				head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting

			if binLocationX < 310:
				msg.angular.z = 0.1
				pub.publish(msg)
				rospy.loginfo("RED FOUND, altering course to right linear=%s angular=%s", msg.linear.x, msg.angular.z)
			elif binLocationX > 330:
				msg.angular.z = -0.1
				pub.publish(msg)
				rospy.loginfo("RED FOUND, altering course to left linear=%s angular=%s", msg.linear.x, msg.angular.z)
			else:
				msg.linear.x = 0.2
				msg.angular.z = 0.0
				pub.publish(msg)
			#if ballLocationY > 420:
				#state = 3
			if depthBin < 0.8:
				state = "bin_found"

		#state for when within distance to drop off object in bin
		#would include arms transformations and reseting arm and camrea
		#positions
		elif state == "bin_found":
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			pub.publish(msg)
			rospy.loginfo("RED FOUND, state 6 achieved STOP linear=%s angular=%s", msg.linear.x, msg.angular.z)
			
			head_joint_positions[1] = 0.0
			trajectory.points[0].positions = head_joint_positions
			trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
			trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
			trajectory.points[0].time_from_start = rospy.Duration(5.0)

			head_goal = FollowJointTrajectoryGoal()
			head_goal.trajectory = trajectory
			head_goal.goal_time_tolerance = rospy.Duration(0.0)

			rospy.loginfo("Setting positions...")
			head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
			#goalDone_cb, active_cb, feedback_cb) # 

			head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
			
			#reset variables when we loop to find new objects
			depthBin = 0
			binLocationX = 0
			binLocationY = 0
			
			depth = 0
			ballLocationX = 0
			ballLocationY = 0
			
			rospy.sleep(3)
			
			state = "find_ball"
		rate.sleep()


if __name__ == '__main__':
	try:
		detectAndRetrieve()
		
	except rospy.ROSInterruptException:
		pass
