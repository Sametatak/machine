#!/usr/bin/env python

import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as efq
from sensor_msgs.msg import Imu
from angles import shortest_angular_distance
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, LaserScan
from demo.msg import PidConfiguration # Make sure to replace 'your_package' with the actual package name
import socket
import json

counter = 0
counter2 = 0

def main():
    global rate
     # Set the rate to 20 Hz
    #imu_sub = rospy.Subscriber("/imu_bosch/data", Imu, imu_callback)
    imu_sub2 = rospy.Subscriber("/imu/data", Imu, imu_callback2)
    rospy.spin()

def get_angle(msg):
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = efq(orientation_list)
    
    return yaw

def imu_callback(data):
    global counter
    yaw1 = get_angle(data)
    counter += 1
    if counter > 40:
        print("bosch-----------------", yaw1)
        counter = 0

      # Sleep to ensure the callback is called at the rate of 20 Hz

def imu_callback2(data):
    global counter2
    yaw = get_angle(data)
    counter2 += 1
    if counter2 > 40:
        print("2---", yaw)
        counter2 = 0
    
      # Sleep to ensure the callback is called at the rate of 20 Hz

if __name__ == "__main__":
    rospy.init_node("stsa")
    main()
