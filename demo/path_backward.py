#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
import time

class GoalRecorder:
    def __init__(self):
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.goals = []
        self.initial_position = None
        self.goal_count = 0
        self.max_goals = 5
        self.is_processing = False
        self.current_goal = None
        self.goal_start_time = None
        self.threshold_distance = 1.0  # 1 meter threshold for reaching the goal
        self.max_time_for_goal = 3  # Maximum 10 seconds to reach a goal
        self.robot_position = None  # To be updated with the robot's current position
        rospy.loginfo("Goal Recorder Node Initialized")

    def goal_callback(self, msg):
        if not self.is_processing:
            if self.initial_position is None:
                self.initial_position = PoseStamped()
                self.initial_position.header = msg.header
                self.initial_position.pose = msg.pose
                rospy.loginfo("Initial position recorded")
            
            self.goals.append(msg)
            self.goal_count += 1
            rospy.loginfo(f"Goal {self.goal_count} recorded")
            
            if self.goal_count >= self.max_goals:
                self.is_processing = True  # Start processing goals
                self.execute_goals()

    def status_callback(self, msg):
        if self.current_goal is None:
            return
        
        # Check if the robot reached the current goal or failed
        for status in msg.status_list:
            if status.status == 3:  # SUCCEEDED
                rospy.loginfo("Goal succeeded")
                self.publish_next_goal()
                return
            elif status.status in [4, 5]:  # ABORTED or REJECTED
                rospy.loginfo("Goal aborted or rejected")
                self.publish_next_goal()
                return
        
        # Check if the robot has been trying for too long
        if time.time() - self.goal_start_time > self.max_time_for_goal:
            rospy.loginfo("Goal time exceeded")
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.goals:
            self.current_goal = self.goals.pop()
            self.goal_start_time = time.time()
            self.goal_pub.publish(self.current_goal)
            rospy.loginfo("Publishing next goal")
        else:
            rospy.loginfo("Returning to initial position")
            if self.initial_position:
                self.current_goal = self.initial_position
                self.goal_start_time = time.time()
                self.goal_pub.publish(self.initial_position)
            self.reset()

    def execute_goals(self):
        rospy.loginfo("Executing goals in reverse order")
        self.publish_next_goal()

    def reset(self):
        rospy.loginfo("Resetting goal recorder for next batch of goals")
        self.goals = []
        self.goal_count = 0
        self.initial_position = None
        self.is_processing = False
        self.current_goal = None

if __name__ == '__main__':
    rospy.init_node('goal_recorder')
    recorder = GoalRecorder()
    rospy.spin()  # Keep the node running and responsive to callbacks
