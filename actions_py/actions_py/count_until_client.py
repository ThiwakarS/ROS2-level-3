#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from casper_interfaces.action import CountUntil
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.action import ActionClient


class CountUntilClientNode(Node):

    def __init__(self):
        super().__init__("count_until_client")

        self.count_until_client_ = ActionClient(
            self, CountUntil, "count_until")

        self.get_logger().info("Count until client started")

    def send_goal(self, target_number, period):
        # wait for the server
        self.count_until_client_.wait_for_server()

        # create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # send the goal
        self.get_logger().info("Sending Goal")
        self.count_until_client_.\
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback).\
            add_done_callback(self.goal_response_callback)

        # get the status of the goal

        # cancel goal timer
        # self.cancel_goal_timer_ = self.create_timer(2, self.cancel_goal_timer_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()

        if (self.goal_handle_.accepted):
            self.get_logger().info("Goal accepted")
            self.goal_handle_.\
                get_result_async().\
                add_done_callback(self.goal_result_callback)
        
        else:
            self.get_logger().warn("Goal rejected")

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if(status == GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().info("Success")
        
        elif(status == GoalStatus.STATUS_ABORTED):
            self.get_logger().error("Aborted")
        
        elif(status == GoalStatus.STATUS_CANCELED):
            self.get_logger().warn("Cancelled")

        self.get_logger().\
            info("Result: " + str(result.reached_number))
        
    def goal_feedback_callback(self, feedback_msg):
        current_number = feedback_msg.feedback.current_number
        self.get_logger().info("Got feedback: " + str(current_number))
    
    # def cancel_goal_timer_callback(self):
    #     self.get_logger().info("Sending a cancel request")
    #     self.goal_handle_.cancel_goal_async()
    #     self.cancel_goal_timer_.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(10, 0.5)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
