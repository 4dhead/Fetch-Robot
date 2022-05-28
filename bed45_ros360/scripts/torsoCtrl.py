#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# callback method triggered when action is complete
def goalDone_cb(state, done):
	rospy.loginfo('goal done')

# callback method triggered when goal becomes active
def active_cb():
	rospy.loginfo('goal active')

# callback method triggered by feedback on action progress from action
def feedback_cb(fb):
	# feedback message contains the third set of variables defined in action message
	rospy.loginfo('goal progress...') #read feedback message


torso_joint_name = ["torso_lift_joint"]
torso_joint_positions = [[0.386], [0.0]]


if __name__ == "__main__":
	rospy.init_node("torso_control")

	rospy.loginfo("Waiting for torso_controller...")
	torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	torso_client.wait_for_server()
	rospy.loginfo("...connected.")
		
	for pose in torso_joint_positions:

		if rospy.is_shutdown():
			break

		trajectory = JointTrajectory()
		trajectory.joint_names = torso_joint_name
		trajectory.points.append(JointTrajectoryPoint())
		trajectory.points[0].positions = pose
		trajectory.points[0].velocities = [0.0] * len(pose)
		trajectory.points[0].accelerations = [0.0] * len(pose)
		trajectory.points[0].time_from_start = rospy.Duration(5.0)

		torso_goal = FollowJointTrajectoryGoal()
		torso_goal.trajectory = trajectory
		torso_goal.goal_time_tolerance = rospy.Duration(0.0)

		rospy.loginfo("Setting positions...")
		torso_client.send_goal(torso_goal) #optioanly add callbacks, note fixed order
			#goalDone_cb, active_cb, feedback_cb) # 

		torso_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
		rospy.loginfo("...done")
