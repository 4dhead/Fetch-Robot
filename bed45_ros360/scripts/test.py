#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

#example based on cv_bridge tutorials
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

bridge = CvBridge()

def cb_depthImage(image):
	global bridge, x, y
	x = 320
	y = 240
	# image msg obtained from callback message for topic
	try:
		cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
		# where x,y is the centre point from the published moment
		depth = cv_image[y][x]
		rospy.logdebug('Depth of point is %s m',depth)
		# For testing/verification:
		cv2.circle(cv_image, (x,y), 10, 255) # draw circle radius 10 at x,y
		cv2.imshow("Image window", cv_image) # display the image
		cv2.waitKey(3)
	except CvBridgeError as e:
		print(e)
if __name__ == '__main__':
	# Initialise the ROS system and become a node
	rospy.init_node('depthTest', anonymous=False)


	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImage)
	rospy.spin()
