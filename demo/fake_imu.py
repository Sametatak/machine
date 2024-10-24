#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

# Define the fixed altitude value you want to set
FIXED_ALTITUDE = 90 # Set your desired altitude here

# Global variable to hold the most recent message
latest_msg = None

def gps_callback(msg):
    global latest_msg
    # Store the received message and modify it in the loop
    latest_msg = msg

def publish_modified_gps():
    # Initialize the ROS node
    rospy.init_node('gps_fix_altitude_and_covariance_modifier')

    # Create a publisher to republish the modified GPS data on the gps/fix topic
    gps_pub = rospy.Publisher('/gps/fix2', NavSatFix, queue_size=10)

    # Subscribe to the original gps/fix topic
    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)

    # Set the rate to 2 Hz
    rate = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        if latest_msg:
            # Modify the altitude to the fixed value
            latest_msg.altitude = FIXED_ALTITUDE

            # Set all covariance values to zero
            latest_msg.position_covariance = [0.0] * 9  # Covariance is a 3x3 matrix, hence 9 elements
            latest_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN  # Optional

            # Republish the modified message
            gps_pub.publish(latest_msg)

        # Sleep to maintain the 2 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_modified_gps()
    except rospy.ROSInterruptException:
        pass
