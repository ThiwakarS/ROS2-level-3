#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
from casper_interfaces.action import TurtleMovement
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class TurtleNode(LifecycleNode):

    def __init__(self):
        super().__init__("turtle_node")
        # server active status
        self.server_active_status = False

        # current turtle name
        self.turtle_name_ = None

        self.cmd_vel_publisher_ = None

        self.kill_client_ = self.create_client(Kill, "/kill")
        self.kill_turtle("turtle1")  # kill the default turtle

        self.get_logger().info("Turtle Node started!")

    def on_configure(self, state):

        # initialisation of services to kill and spawn of turtle
        self.kill_client_ = self.create_client(Kill, "/kill")
        self.spawn_client_ = self.create_client(Spawn, "/spawn")

        # time.sleep(1)
        self.spawn_turtle()  # spawn a new turtle

        # action server things
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        self.turtlesim_movement_server = ActionServer(
            self, TurtleMovement, "/turtlesim_movement",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            # handle_accepted_callback=self.handle_accepted_callback,
            # cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("Turtle Node configured!")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):

        self.kill_turtle(self.turtle_name_)
        self.destroy_client(self.kill_client_)
        self.destroy_client(self.spawn_client_)
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        self.turtlesim_movement_server.destroy()
        self.goal_handle_ = None
        self.server_active_status = False

        self.get_logger().info("Server On cleanup!")

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):

        self.kill_turtle(self.turtle_name_)
        self.destroy_client(self.kill_client_)
        self.destroy_client(self.spawn_client_)
        self.destroy_lifecycle_publisher(self.cmd_vel_publisher_)
        self.turtlesim_movement_server.destroy()
        self.goal_handle_ = None
        self.server_active_status = False

        self.get_logger().info("Server On cleanup!")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):

        # server active status
        self.get_logger().info("Server activated!")
        self.server_active_status = True

        return super().on_activate(state)

    def on_deactivate(self, state):

        # server active status
        self.get_logger().info("Server deactivated!")
        self.server_active_status = False
        return super().on_deactivate(state)

    def goal_callback(self, goal_request: TurtleMovement):
        self.get_logger().info("Got a new goal")

        if (self.goal_handle_ is not None and self.goal_handle_.is_active):
            self.get_logger().info("A goal is already present")
            return GoalResponse.REJECT

        if (self.server_active_status == False):
            self.get_logger().warn("Server not active")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    # def cancel_callback(self, goal_handle: ServerGoalHandle):
    #     pass

    # def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
    #     # self.get_logger().info("Inside HANDLE ACCEPTED callback")
    #     goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):

        result = TurtleMovement.Result()

        if (self.server_active_status == False):
            result.success = False
            result.msg = "Server not active!!!"
            goal_handle.abort()
            self.get_logger().warn("Server not active!!")

            return result
        
        if(not goal_handle.is_active):
            result.success = False
            result.msg = "Goal cancelled / aborted"
            goal_handle.abort()
            self.get_logger().warn("Server not active!!")

            return result

        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # self.get_logger().info("Inside EXECUTE callback")

        linear_vel_x = goal_handle.request.linear_vel_x
        angular_vel_z = goal_handle.request.angular_vel_z
        duration = goal_handle.request.duration

        msg = Twist()
        msg.linear.x = linear_vel_x
        msg.angular.z = angular_vel_z

        curr_time = time.time()

        self.get_logger().info(
            f"publishing on topic : /{self.turtle_name_}/cmd_vel")

        while ((time.time() - curr_time) <= duration):
            self.cmd_vel_publisher_.publish(msg)
            time.sleep(duration / 10)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)

        goal_handle.succeed()
        self.get_logger().info("Successfully executed the goal")

        result.success = True
        result.msg = "Goal executed successfully"

        return result

    def kill_turtle(self, turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service turtle kill...")

        request = Kill.Request()
        request.name = turtle_name
        self.get_logger().info("Killing turtle: " + turtle_name)
        self.kill_client_.call_async(request)

    def spawn_turtle(self, name=None, pos_x=5.455, pos_y=5.455, theta=0.0):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service turtle spawn...")

        request = Spawn.Request()

        if (name != None):
            request.name = name

        request.x = pos_x
        request.y = pos_y
        request.theta = theta

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(self.spawn_turtle_callback)

    def spawn_turtle_callback(self, future):
        try:
            response = future.result()
            self.turtle_name_ = response.name

            # creating a publisher for moving the turtle
            self.cmd_vel_publisher_ = self.create_lifecycle_publisher(
                Twist, f"/{self.turtle_name_}/cmd_vel", 10)

            self.get_logger().info("spawned turtle: " + response.name)
        except Exception as e:
            self.get_logger().error("spawn service call failed %r" % (e, ))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
