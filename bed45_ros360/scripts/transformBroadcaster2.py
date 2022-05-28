#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw#!/usr/bin/env python
import rospy 
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

# The following should be executed every time a depth is calculated
# for an object detected (i.e. from a callback function)

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
