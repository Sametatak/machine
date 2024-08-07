#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def publish_transforms():
    rospy.init_node('tf_publisher')

    br = TransformBroadcaster()

    rate = rospy.Rate(20.0)  # 20 Hz
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Define the transform from map to odom
        transform1 = TransformStamped()
        transform1.header.stamp = current_time
        transform1.header.frame_id = "map"
        transform1.child_frame_id = "odom"
        transform1.transform.translation.x = 0
        transform1.transform.translation.y = 0
        transform1.transform.translation.z = 0.0
        transform1.transform.rotation.x = 0.0
        transform1.transform.rotation.y = 0.0
        transform1.transform.rotation.z = 0.0
        transform1.transform.rotation.w = 1.0

        # Define the transform from odom to base_link
        transform2 = TransformStamped()
        transform2.header.stamp = current_time
        transform2.header.frame_id = "odom"
        transform2.child_frame_id = "base_link"
        transform2.transform.translation.x = 0.0
        transform2.transform.translation.y = 0.0
        transform2.transform.translation.z = 0.0
        transform2.transform.rotation.x = 0.0
        transform2.transform.rotation.y = 0.0
        transform2.transform.rotation.z = 0.0
        transform2.transform.rotation.w = 1.0

        # Define the transform from base_link to base_footprint
        transform3 = TransformStamped()
        transform3.header.stamp = current_time
        transform3.header.frame_id = "base_link"
        transform3.child_frame_id = "base_footprint"
        transform3.transform.translation.x = 0.0
        transform3.transform.translation.y = 0.0
        transform3.transform.translation.z = 0.0
        transform3.transform.rotation.x = 0.0
        transform3.transform.rotation.y = 0.0
        transform3.transform.rotation.z = 0.0
        transform3.transform.rotation.w = 1.0

        # Define the transform from base_link to camera_depth_frame
        transform4 = TransformStamped()
        transform4.header.stamp = current_time
        transform4.header.frame_id = "base_link"
        transform4.child_frame_id = "camera_depth_frame"
        transform4.transform.translation.x = 0.0
        transform4.transform.translation.y = 0.0
        transform4.transform.translation.z = 1.0  # Adjust as needed
        transform4.transform.rotation.x = 0.0
        transform4.transform.rotation.y = 0.0
        transform4.transform.rotation.z = 0.0
        transform4.transform.rotation.w = 1.0

        # Define the transform from camera_imu_optical_frame to camera_link
        transform5 = TransformStamped()
        transform5.header.stamp = current_time
        transform5.header.frame_id = "camera_link"
        transform5.child_frame_id = "camera_imu_optical_frame"
        transform5.transform.translation.x = 0.0
        transform5.transform.translation.y = 0.0
        transform5.transform.translation.z = 0.0
        transform5.transform.rotation.x = 0.0
        transform5.transform.rotation.y = 0.0
        transform5.transform.rotation.z = 0.0
        transform5.transform.rotation.w = 1.0

        # Define the transform from base_link to base_scan
        transform6 = TransformStamped()
        transform6.header.stamp = current_time
        transform6.header.frame_id = "base_link"
        transform6.child_frame_id = "base_scan"
        transform6.transform.translation.x = 0.0
        transform6.transform.translation.y = 0.0
        transform6.transform.translation.z = 0.0
        transform6.transform.rotation.x = 0.0
        transform6.transform.rotation.y = 0.0
        transform6.transform.rotation.z = 0.0
        transform6.transform.rotation.w = 1.0

        # Define the transform from base_link to wheel_left_link
        transform7 = TransformStamped()
        transform7.header.stamp = current_time
        transform7.header.frame_id = "base_link"
        transform7.child_frame_id = "wheel_left_link"
        transform7.transform.translation.x = 0.0
        transform7.transform.translation.y = 0.0
        transform7.transform.translation.z = 0.0
        transform7.transform.rotation.x = 0.0
        transform7.transform.rotation.y = 0.0
        transform7.transform.rotation.z = 0.0
        transform7.transform.rotation.w = 1.0

        # Define the transform from base_link to wheel_right_link
        transform8 = TransformStamped()
        transform8.header.stamp = current_time
        transform8.header.frame_id = "base_link"
        transform8.child_frame_id = "wheel_right_link"
        transform8.transform.translation.x = 0.0
        transform8.transform.translation.y = 0.0
        transform8.transform.translation.z = 0.0
        transform8.transform.rotation.x = 0.0
        transform8.transform.rotation.y = 0.0
        transform8.transform.rotation.z = 0.0
        transform8.transform.rotation.w = 1.0

        # Define the transform from base_link to camera_imu_optical_frame (dynamic)
        transform9 = TransformStamped()
        transform9.header.stamp = current_time
        transform9.header.frame_id = "odom"
        transform9.child_frame_id = "camera_imu_optical_frame"
        transform9.transform.translation.x = 0.0
        transform9.transform.translation.y = 0.0
        transform9.transform.translation.z = 0.0
        transform9.transform.rotation.x = -0.6397669094022864
        transform9.transform.rotation.y = 0.3310504809370483
        transform9.transform.rotation.z = -0.33108645486322996
        transform9.transform.rotation.w = 0.6094962182912164

        # Broadcast the transforms
        br.sendTransform(transform1)
        br.sendTransform(transform2)
        br.sendTransform(transform3)
        br.sendTransform(transform4)
        br.sendTransform(transform5)
        br.sendTransform(transform6)
        br.sendTransform(transform7)
        br.sendTransform(transform8)
        br.sendTransform(transform9)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass

