#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

def fake_gps_publisher():
    # Initialize the ROS node
    rospy.init_node('fake_gps_publisher', anonymous=True)

    # Create a publisher for the /gps/fix topic
    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

    # Set the loop rate (how often to publish the message)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a NavSatFix message object
    gps_msg = NavSatFix()

    # Fill in the NavSatFix message details (latitude, longitude, altitude, etc.)
    gps_msg.header.frame_id = "gps"  # You can set this to your desired frame
    gps_msg.status.status = NavSatStatus.STATUS_FIX
    gps_msg.status.service = NavSatStatus.SERVICE_GPS

    # Set some fake GPS data (latitude, longitude, altitude)
    gps_msg.latitude = 41.1149    # Replace with your desired latitude
    gps_msg.longitude = 29.0116  # Replace with your desired longitude
    gps_msg.altitude = 0.0        # Replace with your desired altitude

    # Set position covariance if needed (optional)
    gps_msg.position_covariance = [0.01, 0, 0,
                                   0, 0.01, 0,
                                   0, 0, 0.01]
    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    # Start publishing the fake GPS data
    while not rospy.is_shutdown():
        gps_msg.header.stamp = rospy.Time.now()
        gps_pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_gps_publisher()
    except rospy.ROSInterruptException:
        pass
