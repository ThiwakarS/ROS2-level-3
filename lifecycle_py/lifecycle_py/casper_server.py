#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from casper_interfaces.action import CasperMovement
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CasperServerNode(LifecycleNode):

    def __init__(self):
        super().__init__("casper_server_node")

        self.goal_handle_: ServerGoalHandle = None
        self.current_position_ = 50
        self.goal_lock_ = threading.Lock()
        self.server_active_status_ = False

    # configuring state of the lifecycle node

    def on_configure(self, state):

        self.declare_parameter("node_name", rclpy.Parameter.Type.STRING)

        self.lifecycle_action_name_ = self.get_parameter("node_name").value

        self.get_logger().info("Current" + str(self.lifecycle_action_name_) + "position: " + str(self.current_position_))

        self.casper_movement_server_ = ActionServer(
            self, CasperMovement, self.lifecycle_action_name_,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_goal_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info("casper server node CONFIGURED")

        return TransitionCallbackReturn.SUCCESS

    # cleanup state of the lifecycle node

    def on_cleanup(self, state):

        self.casper_movement_server_.destroy()
        self.get_logger().info("casper server node CLEANED UP")
        return TransitionCallbackReturn.SUCCESS

    # activate state of the lifecycle node

    def on_activate(self, state):

        self.server_active_status_ = True
        self.get_logger().info("casper server node ACTIVATED")
        return super().on_activate(state)

    # deactivate state of the lifecycle node

    def on_deactivate(self, state):

        self.server_active_status_ = False
        self.get_logger().info("casper server node DEACTIVATED")
        return super().on_activate(state)
    
    # shutdown state of the lifecycle node
    
    def on_shutdown(self, state):

        self.casper_movement_server_.destroy()
        self.get_logger().info("casper server node SHUTDOWNED")
        return super().on_activate(state)



    # accepting or rejecting a goal

    def goal_callback(self, goal_request: CasperMovement.Goal):
        self.get_logger().info("Received the goal request")

        if (self.server_active_status_ == False):
            self.get_logger().warn("SERVER NOT ACTIVE")
            return GoalResponse.REJECT

        # reject invalid goal requests
        if (goal_request.target_position > 100 or
           goal_request.target_position < 0 or
           goal_request.velocity == 0):

            self.get_logger().warn("Invalid goal request")
            return GoalResponse.REJECT

        # Policy preemt existing goal when new goal arrives
        with self.goal_lock_:
            if (self.goal_handle_ is not None and self.goal_handle_.is_active):
                self.get_logger().info("Aborting current goal, Accepting new goal")

                self.goal_handle_.abort()

        self.get_logger().info("Accepted new goal!")
        return GoalResponse.ACCEPT

    # what happens if a cancel request is made
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        if (goal_handle.is_active and self.goal_handle_ is not None):
            return CancelResponse.ACCEPT

        else:
            return CancelResponse.REJECT

    # what to do with the accepted goal
    def handle_accepted_goal_callback(self, goal_handle: ServerGoalHandle):
        goal_handle.execute()

    # actual execution takes place here
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # getting the values from goal request
        target_position = goal_handle.request.target_position
        velocity = goal_handle.request.velocity

        # feedback and result object
        feedback = CasperMovement.Feedback()
        result = CasperMovement.Result()

        # execution of the action
        self.get_logger().info("Goal execution started")

        diff = -1

        while (diff != 0):

            if (self.server_active_status_ == False):
                result.final_position = self.current_position_
                result.message = "Server deactivated mid execution"
                goal_handle.abort()
                return result

            if (not goal_handle.is_active):
                result.final_position = self.current_position_
                result.message = "Current goal aborted"
                goal_handle.abort()
                return result

            if (goal_handle.is_cancel_requested):
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.final_position = self.current_position_
                result.message = "Current goal cancelled"
                return result

            if (self.current_position_ < target_position):
                diff = target_position - self.current_position_
                if (diff >= velocity):
                    self.current_position_ += velocity
                else:
                    self.current_position_ += diff

            else:
                diff = self.current_position_ - target_position
                if (diff >= velocity):
                    self.current_position_ -= velocity
                else:
                    self.current_position_ -= diff

            self.get_logger().info("current position: " + str(self.current_position_))
            feedback.current_position = self.current_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().info("Successfully reached the target position!")

        result.final_position = self.current_position_
        result.message = "Successfully reached the target position"

        return result


def main(args=None):
    rclpy.init(args=args)
    node = CasperServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
