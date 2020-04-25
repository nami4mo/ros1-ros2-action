#! /usr/bin/env python

import rospy
import actionlib
from ros1_action_sample.msg import MoveAction, MoveResult, MoveFeedback
 

class SampleActionServer(object):

    def __init__(self):
        self._as = actionlib.SimpleActionServer('move', MoveAction, execute_cb=self.execute_cb, auto_start=False)
        rospy.loginfo('server start!')
        self._as.start()
    

    def execute_cb(self, goal):
        result = MoveResult()
        move_name = goal.command
        move_count = goal.count
        rospy.loginfo('receive goal: %s' % move_name)
        r = rospy.Rate(1) # 1Hz
        for i in range(move_count):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s end... (preempted)' % move_name)
                self._as.set_preempted()
                return
            # do some moves here
            rospy.loginfo('%s now! (count: %d)' % (move_name, i))
            r.sleep()
        
        rospy.loginfo('%s: end!! (succeeded)' % move_name)
        result.result = 'success!'
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('action_server_node')
    SampleActionServer()
    rospy.spin()