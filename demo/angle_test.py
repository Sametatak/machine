#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf

def quaternion_to_degrees(quaternion):
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    
    # Extract the yaw (rotation around z-axis) in radians
    yaw_radians = euler[2]

    # Convert yaw to degrees (0 to 360), with 0 meaning north
    yaw_degrees = (yaw_radians * 180.0 / 3.14159) % 360
    return yaw_degrees

def imu_callback(data):
    # Convert IMU quaternion to degrees
    yaw_degrees = quaternion_to_degrees(data.orientation)
    
    # Publish the yaw angle in degrees
    pub.publish(yaw_degrees)
    print(yaw_degrees)
if __name__ == '__main__':
    rospy.init_node('imu_to_degrees', anonymous=True)

    # Publisher to publish the yaw in degrees
    pub = rospy.Publisher('/imu/yaw_degrees', Float32, queue_size=10)
    
    # Subscriber to the IMU data
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    # Keep the node running
    rospy.spin()

