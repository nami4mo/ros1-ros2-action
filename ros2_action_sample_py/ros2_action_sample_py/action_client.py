import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_msgs.action import Move


class SampleActionClient(Node):
    def __init__(self):
        super().__init__('action_client_node')
        self._ac = ActionClient(self, Move, 'move')
        self._ac.wait_for_server()
        self._send_move_goals()


    def _send_move_goals(self):
        goal = Move.Goal()
        
        goal.command = 'dance'
        goal.count = 5
        self._ac.send_goal_async(goal)
        time.sleep(2.5)

        goal.command = 'run'
        goal.count = 5
        self._ac.send_goal_async(goal)
        time.sleep(2.5)

        goal.command = 'walk'
        goal.count = 5
        self._ac.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = SampleActionClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()