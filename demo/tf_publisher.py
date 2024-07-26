#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

def publish_transforms():
    rospy.init_node('tf_publisher')

    br = TransformBroadcaster()

    rate = rospy.Rate(20.0)  # 10 Hz
    while not rospy.is_shutdown():
        # Define the transform from map to odom
        transform1 = TransformStamped()
        transform1.header.stamp = rospy.Time.now()
        transform1.header.frame_id = "map"
        transform1.child_frame_id = "odom"
        transform1.transform.translation.x = 0
        transform1.transform.translation.y = 0
        transform1.transform.translation.z = 0.0
        transform1.transform.rotation.x = 0.0
        transform1.transform.rotation.y = 0.0
        transform1.transform.rotation.z = 0
        transform1.transform.rotation.w = 1

        # Define the transform from odom to base_link
        transform2 = TransformStamped()
        transform2.header.stamp = rospy.Time.now()
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
        transform3.header.stamp = rospy.Time.now()
        transform3.header.frame_id = "base_link"
        transform3.child_frame_id = "base_footprint"
        transform3.transform.translation.x = 0.0
        transform3.transform.translation.y = 0.0
        transform3.transform.translation.z = 0.0
        transform3.transform.rotation.x = 0.0
        transform3.transform.rotation.y = 0.0
        transform3.transform.rotation.z = 0.0
        transform3.transform.rotation.w = 1.0

        # Broadcast the transforms
        br.sendTransform(transform1)
        br.sendTransform(transform2)
        br.sendTransform(transform3)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
