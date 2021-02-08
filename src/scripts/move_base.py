#!/usr/bin/env python

#Code is inspired by http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals (written in C++).
#TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class MoveRobotToGoal():
	@staticmethod

	#position of goal (in our case, the weed location) is passed to the move2goal function.
	#This works using move_base
	def move2goal(point):
		# the translation from the sprayer to the base_link is different only in the x-axis by 0.450
		sprayer_tf = 0.450

		# initialize the move_base action
		move_base = actionlib.SimpleActionClient("/thorvald_001/move_base", MoveBaseAction)
		
		move_base.wait_for_server()

		goal = MoveBaseGoal()

		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = point.pose.position.x + sprayer_tf
		goal.target_pose.pose.position.y = point.pose.position.y
		goal.target_pose.pose.position.z = point.pose.position.z


		#start moving
		move_base.send_goal(goal)

		# check for the status of the movement
		success = move_base.wait_for_result() 

		# print to the screen regardless of the output (success or failure)
		if not success:
			move_base.cancel_goal()
			rospy.loginfo("The base failed to move to the weed location")   
		else:
			rospy.loginfo("Robot moved to the weed location")




