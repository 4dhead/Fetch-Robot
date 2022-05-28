#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def cb_depthImage(image):
	global bridge
	global x
	global y
	
	try:
		cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
		depth = cv_image[y][x]
		rospy.logdebug('Depth of point is %s m', depth)
		
		cv2.circle(cv_image, (x,y), 10, 255)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
	except CvBridgeError as e:
		print (e)
		
if __name__ == '__main__':
	rospy.init_node('depth_test',anonymous=False)
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImage)
	rospy.spin()
