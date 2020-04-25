#! /usr/bin/env python

import rospy
import roslib
import actionlib
from ros1_action_sample.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal


class SampleActionClient(object):
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('move', MoveAction)
        self._ac.wait_for_server()
        self._send_move_goals()


    def _send_move_goals(self):
        goal = MoveGoal()
        
        goal.command = 'dance'
        goal.count = 5
        self._ac.send_goal(goal)
        rospy.sleep(2.5)

        goal.command = 'run'
        goal.count = 5
        self._ac.send_goal(goal)
        rospy.sleep(2.5)

        goal.command = 'walk'
        goal.count = 5
        self._ac.send_goal(goal)


if __name__ == '__main__':
    rospy.init_node('action_client_node')
    SampleActionClient()
    rospy.spin()