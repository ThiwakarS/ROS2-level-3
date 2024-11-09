#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from casper_interfaces.action import CountUntil
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CountUntilServerNode(Node):

    def __init__(self):
        super().__init__("count_until_server")

        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.count_until_server_ = ActionServer(
            self, CountUntil, "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info("Count Until Action server started")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received the goal request")

        # 1) Policy: refuse new goal if current goal is still active
        # only runs one goal at a time, only the first one.
        # all the new ones will be rejected until the current goal finishes

        # with self.goal_lock_:
        #     if (self.goal_handle_ is not None and
        #        self.goal_handle_.is_active):
        #         self.get_logger().info("A goal is already active rejecting new goal")
        #         return GoalResponse.REJECT

        # 2) Policy : preemt existing goal when receiving new goal
        # basically trash the currently running goal and run the new goal
        # only the newest goal runs, all the previous ones are aborted

        # with self.goal_lock_:
        #     if(self.goal_handle_ is not None and self.goal_handle_.is_active):
        #         self.get_logger().info("Abort current goal and accept new goal")
        #         self.goal_handle_.abort()


        # validate the goal
        if (goal_request.target_number <= 0):
            self.get_logger().info("Goal rejected")
            return GoalResponse.REJECT

        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        return CancelResponse.ACCEPT  # or REJECT
    
    def handle_accepted_goal_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if(self.goal_handle_ is not None):
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):

        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # feedback section
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        # execute the action
        self.get_logger().info("Goal execution started")
        counter = 0
        for i in range(target_number):

            if(not goal_handle.is_active): 
                # if the goal is aborted we need
                # to check that its not active and
                # return the result
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result

            if (goal_handle.is_cancel_requested):
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result

            counter += 1
            self.get_logger().info(str(counter))

            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # final state of the goal
        goal_handle.succeed()
        self.get_logger().info("Goal reached")

        # return the goal
        result.reached_number = counter
        self.process_next_goal_in_queue()

        return result

    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if(len(self.goal_queue_) > 0):
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
