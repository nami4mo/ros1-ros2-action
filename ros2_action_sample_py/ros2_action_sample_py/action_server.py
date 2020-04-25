# modified by NS

# https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from my_msgs.action import Move
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class SampleActionServer(Node):
    def __init__(self):
        super().__init__('action_server_node')

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._execute_lock = threading.Lock()

        self._as = ActionServer(
            self,
            Move,
            'move',
            execute_callback=self.execute_callback, 
            # goal_callback=self.goal_callback,
            # handle_accepted_callback=self.handle_accepted_callback,
            # cancel_callback=self.cancel_callback,
            # callback_group=ReentrantCallbackGroup()
        )


    def execute_callback(self, goal_handle):
        # with self._execute_lock:
        result = Move.Result()
        move_name = goal_handle.request.command
        move_count = goal_handle.request.count
        self.get_logger().info(f'receive goal: {move_name}')
        for i in range(move_count):
            if not goal_handle.is_active:
                self.get_logger().info(f'{move_name} end... (preempted)')
                result = Move.Result()
                return result
            # do some moves here
            self.get_logger().info(f'{move_name} now! (count: {i})')
            # 私のROS2(rclpy)環境には、ROS1のRate相当のものがまだないのでこれで仮置き
            # 実際は、Node.create_rate()というものがあるらしい
            time.sleep(1.0)

        self.get_logger().info(f'{move_name} end!! (succeeded)')
        goal_handle.succeed()
        return result


    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    def handle_accepted_callback(self, goal_handle):
        # self.get_logger().info('Received accept request')
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the existing goal
                # self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()


    def cancel_callback(self, goal):
        """Accepts or rejects a client request to cancel an action."""
        # self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init()
    node = SampleActionServer()

    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()