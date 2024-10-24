#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import random

def fake_laser_scan_publisher():
    rospy.init_node('fake_laser_scan_publisher', anonymous=True)
    scan_pub = rospy.Publisher('front/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize the LaserScan message
    scan = LaserScan()
    scan.header.frame_id = 'laser_frame'  # Set this to your laser's frame
    scan.angle_min = -math.pi / 2  # -90 degrees
    scan.angle_max = math.pi / 2   # 90 degrees
    scan.angle_increment = math.pi / 180  # 1 degree increments
    scan.time_increment = 0.0
    scan.scan_time = 0.1  # 10 Hz
    scan.range_min = 0.2
    scan.range_max = 10.0

    num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)

    while not rospy.is_shutdown():
        scan.header.stamp = rospy.Time.now()

        # Generate fake range data (simulate obstacles)
        scan.ranges = []
        scan.intensities = []
        for i in range(num_readings):
            # Simulate a wall at 5 meters with some random noise
            fake_range = 5.0 + random.uniform(-0.1, 0.1)
            scan.ranges.append(fake_range)
            scan.intensities.append(1.0)  # Constant intensity

        # Publish the fake LaserScan message
        scan_pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_laser_scan_publisher()
    except rospy.ROSInterruptException:
        pass

