#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from casper_interfaces.action import CasperMovement
from std_msgs.msg import Empty
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.action import ActionClient
import random

class CasperClientNode(Node):

    def __init__(self):
        super().__init__("casper_client")

        self.goal_handle_ : ClientGoalHandle = None
        self.cancel_move_subscriber_ = self.create_subscription(Empty, "cancel_move", self.cancel_move_subscription_callback, 10)
        self.casper_client_ = ActionClient(
            self, CasperMovement, "casper_movement")
        
        self.send_goal_timer_ = self.create_timer(10, self.send_goal_timer_callback)

        self.get_logger().info("Casper client started")

    def send_goal_timer_callback(self):
        # wait for server
        self.casper_client_.wait_for_server()

        # create a goal
        goal = CasperMovement.Goal()
        goal.target_position = random.randint(0, 100)
        goal.velocity = random.randint(0, 5)

        # send the goal
        self.get_logger().info("Sending the goal to the server")
        self.get_logger().info(str(goal.target_position) + " " + str(goal.velocity))
        self.casper_client_.\
            send_goal_async(goal, feedback_callback=self.feedback_callback).\
            add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        
        self.goal_handle_: ClientGoalHandle = future.result()

        if(self.goal_handle_.accepted):
            self.get_logger().info("Goal accepted!")
            self.goal_handle_.\
                get_result_async().\
                add_done_callback(self.goal_result_callback)
        
        else:
            self.get_logger().info("Goal rejected")        


    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if(status == GoalStatus.STATUS_ACCEPTED):
            self.get_logger().info("Success")
        
        elif(status == GoalStatus.STATUS_ABORTED):
            self.get_logger().error("Aborted")
        
        elif(status == GoalStatus.STATUS_CANCELED):
            self.get_logger().warn("Cancelled")
        
        self.get_logger().info("Result position: " + \
                               str(result.final_position))
        self.get_logger().info("Result message: " + result.message)

    def feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info("Feedback current position: " \
                               + str(current_position))
    
    def cancel_move_subscription_callback(self, msg):
        if(self.goal_handle_ is not None):
            self.get_logger().info("Sending cancel request")
            self.goal_handle_.cancel_goal_async()
        
        else:
            self.get_logger().info("Invalid Cancel Request")
            


def main(args=None):
    rclpy.init(args=args)
    node = CasperClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()