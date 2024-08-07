#!/usr/bin/env python

import rospy
import time
import scipy 
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as efq
from sensor_msgs.msg import Imu
from angles import shortest_angular_distance
from std_msgs.msg import Float32
import math
from sensor_msgs.msg import Image , LaserScan
from dynamic_reconfigure.server import Server
from demo.cfg import PIDConfig

MAX_ANG_VEL = 4.5

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

class Rotate():
   def __init__(self):
      print("Start rotate state")
      msg = rospy.wait_for_message("/imu/data", Imu)
      
      self.controller = PIDController(2.0, 0.0001, 0.8)
      self.srv = Server(PIDConfig, self.dynamic_reconfigure_callback)

      initial_yaw = self.get_angle(msg)
      self.yaw = 0.0
      self.target_angle = initial_yaw
      self.angle_update = 0.0

      cmd_pub = rospy.Publisher("/move_base/cmd_vel", Twist, queue_size=1)
      float_sub = rospy.Subscriber("/float_value", Float32, self.float_callback)
      imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
      
      angle_region_threshold = 0.2 
      
      print("initial_yaw", initial_yaw, "target_yaw", self.target_angle)

      last_time = rospy.Time.now()
      rate = rospy.Rate(20)
      while not rospy.is_shutdown():
         self.target_angle += self.angle_update
         print(self.angle_update )
         self.angle_update = 0
         diff = shortest_angular_distance(self.yaw, self.target_angle)
         print("yaw", self.yaw, "target", self.target_angle, "err", diff)
         if abs(diff) >= angle_region_threshold:
             last_time = rospy.Time.now()
             
         vel = Twist()
         vel.angular.z = min(MAX_ANG_VEL, self.controller.update(0.0, diff*3/2, 1.0 / 20.0))
         vel.angular.z = max(-MAX_ANG_VEL, vel.angular.z)
         if 0.1<vel.angular.z<0.3:
            vel.angular.z = 0.5
         elif -0.1>vel.angular.z>-0.3:
            vel.angular.z = -0.5
         cmd_pub.publish(vel)
         rate.sleep()

   def dynamic_reconfigure_callback(self, config, level):
      self.controller.kp = config.kp
      self.controller.ki = config.ki
      self.controller.kd = config.kd
      rospy.loginfo("Reconfigure Request: kp={kp}, ki={ki}, kd={kd}".format(**config))
      return config

   def get_angle(self, msg):
      orientation_q = msg.orientation
      orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      (roll, pitch, yaw) = efq(orientation_list)
      
      return yaw 

   def imu_callback(self, data):
      self.yaw = self.get_angle(data)
      
   def float_callback(self, data):
      value = data.data
      
      self.angle_update = value/180*math.pi

if __name__ == '__main__':
   rospy.init_node("stsa")
   Rotate()
