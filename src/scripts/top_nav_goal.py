#! /usr/bin/env python

import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('topological_navigation_node')

    # call the client and wait for the server to be available
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

    # send goal to move from Waypoint 3 to Waypoint 13, and back to the initial waypoint, WayPoint1
    goal = GotoNodeGoal()
    goal.target = "WayPoint3"
    client.send_goal(goal)
    client.wait_for_result() 

    goal = GotoNodeGoal()
    goal.target = "WayPoint5"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint7"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint9"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint10"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint11"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint12"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint13"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint10"
    client.send_goal(goal)
    client.wait_for_result()

    goal = GotoNodeGoal()
    goal.target = "WayPoint1"
    client.send_goal(goal)
    client.wait_for_result()
    