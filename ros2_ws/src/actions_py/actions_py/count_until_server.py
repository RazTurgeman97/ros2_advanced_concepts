#!/usr/bin/env python3
import rclpy
import time
import threading # To use goal lock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        
        self.goal_handle_: ServerGoalHandle = None    # initiating an empty goal handle
        self.goal_lock_ = threading.Lock()   # Search about threading library and thread locking.
        self.goal_queue_ = [] # Goal queue list
        
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback = self.goal_callback,
            handle_accepted_callback = self.handle_accepted_callback,
            cancel_callback = self.cancel_callback,
            execute_callback = self.execute_callback,
            callback_group = ReentrantCallbackGroup()
        )
        self.get_logger().info("Action Server has been started.")
        
    def goal_callback(self, goal_request: ServerGoalHandle):
        
        self.get_logger().info("Received a goal.")
        
        # # Policy: refuse new goal if current goal is still active
        # with self.goal_lock_: # making sure this variable is being accessed once at a time.
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, Rejecting new goal.")
        #         return GoalResponse.REJECT
        
        
        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting goal. Invalid target number.")
            return GoalResponse.REJECT
        
        # # Policy: preempt existing goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and access new goal.")
        #         self.goal_handle_.abort()
                
            
        self.get_logger().info("Accepting goal.")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
                self.get_logger().info("Another goal is being executed at the moment.\nThe goal has been added the queue and will be executed when the current executing is finished.")
            else:
                goal_handle.execute()
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        
        self.get_logger().info("Received a cancel request.")
        return CancelResponse.ACCEPT # or REJECT
        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        with self.goal_lock_: # making sure this variable is being accessed once at a time.
            self.goal_handle_ = goal_handle
        
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        # Execute the action
        self.get_logger().info("Executing the goal...")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active: # keep checking if goal was interrupted and aborted.
                result.reached_number = counter
                self.process_next_goal_in_queue() # check for queue goals.
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal.")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue() # check for queue goals.
                return result
            counter +=1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
            
        # Once done, set goal final state
        goal_handle.succeed()
        
        # If aborted
        # goal_handle.abort()
        
        # and send the result
        result = CountUntil.Result()
        result.reached_number = counter
        self.process_next_goal_in_queue() # check for queue goals.
        return result
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute() # Remove the element for the list and execute.
            else:
                self.goal_handle_ = None
            

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
