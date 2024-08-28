#!/usr/bin/env python

import rospy
import time
import scipy 
import numpy as np
import math
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

MAX_ANG_VEL = 1

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

class PIDData:
    def __init__(self, data1, data2):
        self.data1 = data1
        self.data2 = data2

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

class Rotate():
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 10001))
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)  # Set timeout for the socket

        print("Start rotate state")
        msg = rospy.wait_for_message("/imu_bosch/data", Imu)
        
        self.controller = PIDController(2.5, 0.0000, 1.1)
        initial_yaw = self.get_angle(msg)
        self.yaw = 0.0
        self.target_angle = initial_yaw
        self.angle_update = 0.0

        self.cmd_pub = rospy.Publisher("/move_base/cmd_vel", Twist, queue_size=1)
        float_sub = rospy.Subscriber("/float_value", Float32, self.float_callback)
        imu_sub = rospy.Subscriber("/imu_bosch/data", Imu, self.imu_callback)
        pid_sub = rospy.Subscriber("/pid_params", PidConfiguration, self.pid_callback)
        
        angle_region_threshold = 0.2
        
        print("initial_yaw", initial_yaw, "target_yaw", self.target_angle)

        last_time = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.tcp_listen()
            self.target_angle = 0
            self.angle_update = 0
            diff = shortest_angular_distance(self.yaw, self.target_angle)
            print("yaw", self.yaw *180/math.pi )
            if abs(diff) >= angle_region_threshold:
                last_time = rospy.Time.now()
                
            vel = Twist()
            vel.angular.z = min(MAX_ANG_VEL, self.controller.update(0.0, diff * 3 / 2, 1.0 / 20.0))
            vel.angular.z = max(-MAX_ANG_VEL, vel.angular.z)
            if -0.7 < vel.angular.z < 0.4:
                vel.angular.z = 0.4
            elif -0.7 > vel.angular.z > -0.4:
                vel.angular.z = -0.4
            if abs(diff) < angle_region_threshold:
                vel.angular.z = 0
            self.cmd_pub.publish(vel)
            rate.sleep()
            
    def get_angle(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = efq(orientation_list)
        
        return yaw 

    def imu_callback(self, data):
        self.yaw = self.get_angle(data)
        
    def float_callback(self, data):
        value = data.data
        self.angle_update = value / 180 * math.pi
    
    def pid_callback(self, data):
        self.controller.set_params(data.kp, data.ki, data.kd)
        print(data.kp, data.ki, data.kd)

    def tcp_listen(self):
        try:
            client_socket, client_address = self.server_socket.accept()
            print(f"Connection from {client_address}")

            data = client_socket.recv(1024)
            if data:
                json_data = data.decode('utf-8')
                print(f"Received data: {json_data}")

                # JSON verisini ayrıştırın
                parsed_data = json.loads(json_data)
                pid_data = PIDData(
                    data1=PID(parsed_data['data1']['kp'], parsed_data['data1']['ki'], parsed_data['data1']['kd']),
                    data2=PID(parsed_data['data2']['kp'], parsed_data['data2']['ki'], parsed_data['data2']['kd'])
                )

                # Değerleri ekranda yazdırın
                print(f"data1 -> kp: {pid_data.data1.kd}, ki: {pid_data.data1.ki}, kd: {pid_data.data1.kp}")
                print(f"data2 -> kp: {pid_data.data2.kd}, ki: {pid_data.data2.ki}, kd: {pid_data.data2.kp}")
                self.controller.set_params(pid_data.data1.kd, pid_data.data1.ki, pid_data.data1.kp)
                
            client_socket.close()
        except socket.timeout:
            pass  # Do nothing if no connection attempt was made within the timeout period

if __name__ == '__main__':
    rospy.init_node("stsa")
    Rotate()
