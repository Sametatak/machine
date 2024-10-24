#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthToLaser:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.image_callback)
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.angle_min = -1.57  # Example values, adjust according to your sensor's specs
        self.angle_max = 1.57
        self.angle_increment = 0.01
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 10.0

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        # Convert depth image to LaserScan
        height, width = cv_image.shape
        middle_row = cv_image[height // 2, :]
        ranges = np.clip(middle_row, self.range_min, self.range_max)

        scan = LaserScan()
        scan.header = data.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.scan_pub.publish(scan)

if __name__ == '__main__':
    rospy.init_node('depth_to_laserscan')
    dtl = DepthToLaser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

