#!/usr/bin/env python

import rospy
import time
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from angles import shortest_angular_distance
from tf.transformations import euler_from_quaternion as efq
 


MAX_ANG_VEL = 4.5

class PIDController:
    def __init__(self):
        self.kp = rospy.get_param('~kp', 2.0)
        self.ki = rospy.get_param('~ki', 0.0001)
        self.kd = rospy.get_param('~kd', 0.8)

        self.__integral = 0.0
        self.prev_error = 0.0

    def update(self, actual, target, dt):
        error = target - actual
        d = self.kd * (error - self.prev_error) / dt
        self.__integral += self.ki * error * dt
        self.prev_error = error

        return self.__integral + self.kp*error + d

class Rotate():
    def __init__(self):
        rospy.init_node("stsa", anonymous=True)
        self.controller = PIDController()
        self.cmd_pub = rospy.Publisher("/move_base/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/float_value", Float32, self.float_callback)

        self.yaw = 0.0
        self.target_angle = 0.0
        self.angle_update = 0.0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_pid_params()
            self.target_angle += self.angle_update
            self.angle_update = 0
            diff = shortest_angular_distance(self.yaw, self.target_angle)
            vel = Twist()
            vel.angular.z = min(MAX_ANG_VEL, self.controller.update(0.0, diff*3/2, 1.0 / 20.0))
            vel.angular.z = max(-MAX_ANG_VEL, vel.angular.z)
            if 0.1 < vel.angular.z < 0.3:
                vel.angular.z = 0.5
            elif -0.1 > vel.angular.z > -0.3:
                vel.angular.z = -0.5
            self.cmd_pub.publish(vel)
            rate.sleep()

    def update_pid_params(self):
        self.controller.kp = rospy.get_param('~kp', self.controller.kp)
        self.controller.ki = rospy.get_param('~ki', self.controller.ki)
        self.controller.kd = rospy.get_param('~kd', self.controller.kd)
        print(self.controller.kd,self.controller.ki,self.controller.kp)
    def imu_callback(self, data):
        self.yaw = self.get_angle(data)

    def float_callback(self, data):
        value = data.data
        self.angle_update = value / 180 * math.pi

    def get_angle(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = efq(orientation_list)
        return yaw

if __name__ == '__main__':
    Rotate()
