#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def tf_callback(trans, pub):
    # Filter out the transform between odom and camera_imu_optical_frame
    if trans.header.frame_id == "odom" and trans.child_frame_id == "camera_imu_optical_frame":
        rospy.loginfo("Ignoring transform from odom to camera_imu_optical_frame")
        return
    
    # If it's a transform we want, publish it to a new topic
    rospy.loginfo("Publishing filtered transform from %s to %s", trans.header.frame_id, trans.child_frame_id)
    pub.publish(trans)

if __name__ == "__main__":
    rospy.init_node("tf_listener_filter_publisher")

    # Create a tf2 Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a publisher to publish filtered transforms
    pub = rospy.Publisher('/filtered_tf', TransformStamped, queue_size=10)

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Retrieve the transform from source_frame to target_frame
            trans = tf_buffer.lookup_transform('odom', 'camera_imu_optical_frame', rospy.Time())

            # Filter and publish the transform
            tf_callback(trans, pub)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()

