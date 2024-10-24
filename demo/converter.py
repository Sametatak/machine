#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np

class DepthImageToPointCloud:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('depth_image_to_pointcloud', anonymous=True)

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Downsampling factor (e.g., 2 will reduce resolution by half)
        self.downsample_factor = rospy.get_param('~downsample_factor', 1)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        # Publisher
        self.pointcloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=1)

    def camera_info_callback(self, msg):
        # Extract and adjust intrinsic camera parameters for downsampling
        self.fx = msg.K[0] / self.downsample_factor
        self.fy = msg.K[4] / self.downsample_factor
        self.cx = msg.K[2] / self.downsample_factor
        self.cy = msg.K[5] / self.downsample_factor
        rospy.loginfo("Camera intrinsic parameters received and adjusted for downsampling.")
        # Unsubscribe after getting the parameters
        self.camera_info_sub.unregister()

    def depth_image_callback(self, msg):
        if self.fx is None:
            rospy.logwarn("Waiting for camera intrinsic parameters.")
            return

        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array /= 1000.0  # Convert from mm to meters if necessary

        # Downsample the depth image
        depth_array = depth_array[::self.downsample_factor, ::self.downsample_factor]

        height, width = depth_array.shape
        rospy.loginfo("Depth image received: width=%d, height=%d", width, height)

        # Create coordinate grid adjusted for downsampling
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
        u_coords = u_coords * self.downsample_factor
        v_coords = v_coords * self.downsample_factor

        # Compute the x, y, z coordinates
        x = (u_coords - self.cx) * depth_array / self.fx
        y = (v_coords - self.cy) * depth_array / self.fy
        z = depth_array

        # Stack and reshape to list of points
        points = np.dstack((x, y, z)).reshape(-1, 3)

        # Remove invalid points (where depth is zero or NaN)
        valid_points = points[~np.isnan(points).any(axis=1) & (points[:, 2] > 0)]

        # Create PointCloud2 message
        header = msg.header
        pointcloud_msg = pc2.create_cloud_xyz32(header, valid_points)

        # Publish the point cloud
        self.pointcloud_pub.publish(pointcloud_msg)
        rospy.loginfo("Point cloud published with %d points.", len(valid_points))

if __name__ == '__main__':
    try:
        node = DepthImageToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
