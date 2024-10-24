#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
from angles import shortest_angular_distance
from tf.transformations import euler_from_quaternion as efq

ANGLE_REGION_THRESHOLD = 0.1
MAX_ANG_VEL = 4
MAX_VEL_LIN = 4
class PIDController: 
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.__integral = 0.0
        self.prev_error = 0.0
    
    def update(self, actual, target, dt):
        error = target - actual
        d = self.kd * (error - self.prev_error) / dt
        self.__integral += self.ki * error * dt
        self.prev_error = error
        
        return self.__integral + self.kp*error + d
    
    def set_params(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd



class PoseFollower:
    def __init__(self):
        rospy.init_node('pose_follower')

        self.pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.pose_callback)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/move_base/cmd_vel', Twist, queue_size=10)
        self.yaw = 0.0
        self.target_pose = None
        self.current_pose = None
        self.controller_angle = PIDController(1.50, 0.0000, 0.8)
        self.controller_lin = PIDController(1.50, 0.001, 0.8)
        self.angle_to_goal = 0
    def pose_callback(self, msg):
        self.target_pose = msg.pose

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.yaw = self.get_yaw(self.current_pose)
        #print(self.current_pose.position.x,self.current_pose.position.y)
        self.update_velocity()

    def update_velocity(self):
        if self.target_pose is None or self.current_pose is None:
            return

        twist = Twist()
        distance = self.calculate_distance(self.current_pose, self.target_pose)

        if distance > 1:  # tolerance
            self.angle_to_goal = math.atan2(self.target_pose.position.y - self.current_pose.position.y,
                                       self.target_pose.position.x - self.current_pose.position.x)
            print(self.angle_to_goal)
            twist.linear.x = self.controller_lin.update(0.0, distance/3, 1.0 / 20.0)
            twist.angular.z = self.update_angeler_vel()

            if twist.linear.x > MAX_VEL_LIN:
                twist.linear.x =  MAX_VEL_LIN
            if twist.angular.z > 1.0:
                twist.angular.z = 1.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
    
    def update_angeler_vel(self):
        
        #print(self.angle_update )
        
        diff = shortest_angular_distance(self.yaw, self.angle_to_goal)
        #print("yaw", self.yaw, "target", self.angle_to_gaol, "err", diff)
        if abs(diff) >= ANGLE_REGION_THRESHOLD:
            last_time = rospy.Time.now()
         
        vel = Twist()
        vel.angular.z = min(MAX_ANG_VEL, self.controller_angle.update(0.0, diff, 1.0 / 20.0))
        vel.angular.z = max(-MAX_ANG_VEL, vel.angular.z)
        return vel.angular.z

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 +
                         (pose1.position.y - pose2.position.y) ** 2)

    def get_yaw(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = efq(orientation_list)

        return yaw 

if __name__ == '__main__':
    try:
        PoseFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
