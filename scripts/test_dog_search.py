#! /usr/bin/env python
import roslib; roslib.load_manifest('dogsim')
import rospy
import actionlib

from dogsim.msg import *

if __name__ == '__main__':
    rospy.init_node('test_dog_search')
    client = actionlib.SimpleActionClient('focus_head_action', FocusHeadAction)
    client.wait_for_server()

    goal = FocusHeadGoal()
    goal.target = 'dog';
    goal.isPositionSet = False;

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
