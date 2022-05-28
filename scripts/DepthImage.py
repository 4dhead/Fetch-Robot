#!/usr/bin/env python
#example based on cv_bridge tutorials
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from opencv_apps.msg import MomentArrayStamped
from geometry_msgs.msg import TransformStamped
import rospy
import tf2_ros
import tf_conversions
import cv2

bridge = CvBridge()

greenLocationX = 0
greenLocationY = 0

def MomentArrayStampedMessageRecieved(msg):
	global greenLocationX
	global greenLocationY
	
	try:
		rospy.loginfo("Green Found At :X: %.2f, Y: %.2f",msg.moments[0].center.x,msg.moments[0].center.y)
		greenLocationX = msg.moments[0].center.x
		greenLocationY = msg.moments[0].center.y
	except Exception as e:
		rospy.loginfo("No Green Detected")


def cb_depthImage(image):
	global bridge, x, y
	x = int(greenLocationX)
	y = int(greenLocationY)
	rospy.loginfo('x %s',x)
	rospy.loginfo('y %s',y)
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
	except CvBridgeError as e:
		print(e)

def transformBroadcaster():
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "head_camera_depth_frame"
	t.child_frame_id = "target_object"
	t.transform.translation.x = depth
	t.transform.translation.y = 0 #x_offset
	t.transform.translation.z = 0 #y_offset
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0) # no rotation
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]
	br.sendTransform(t)


if __name__ == '__main__':
	# Initialise the ROS system and become a node
	rospy.init_node('depthTest', anonymous=False)
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImage)
	rospy.Subscriber('/contour_moments/moments', MomentArrayStamped, MomentArrayStampedMessageRecieved)
	#transformBroadcaster
	rospy.spin()
