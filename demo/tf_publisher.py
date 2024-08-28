#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def gps_publisher():
    rospy.init_node('gps_navsatfix_publisher', anonymous=True)
    pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
    rate = rospy.Rate(1)  # Publish rate in Hz

    # Starting GPS coordinates (example: somewhere in Istanbul)
    latitude = 41.015137
    longitude = 28.979530
    altitude = 100.0

    while not rospy.is_shutdown():
        gps_msg = NavSatFix()
        
        # Fill in the header (you can set the frame_id as needed)
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "base_link"
        
        # Fill in the latitude, longitude, and altitude
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude

        # You can fill in other fields like position_covariance if needed
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        rospy.loginfo("Publishing GPS Data: Lat: %f, Lon: %f, Alt: %f", 
                      gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
        
        pub.publish(gps_msg)
        rate.sleep()

        # Slightly increment latitude and longitude to create points very close to each other
        latitude += 0.000001  # Increment latitude by a small amount
        longitude += 0.000001  # Increment longitude by a small amount

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

