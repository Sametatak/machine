#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import NavSatFix
import utm
import math

class InitialPosePublisher:
    def __init__(self):
        rospy.init_node('initialpose_publisher', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        self.position = None
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)  # Get yaw offset parameter

    def gps_callback(self, data):
        # Extract position from GPS fix
        if not math.isnan(data.latitude) and not math.isnan(data.longitude):
            self.position = (data.latitude, data.longitude)
            
            # Convert GPS to UTM coordinates
            utm_goal = utm.from_latlon(self.position[0], self.position[1])
            goal_easting = utm_goal[0]
            goal_northing = utm_goal[1]
            
            # Broadcast the transform from 'utm' to 'gps'
            relative_position = (goal_easting, goal_northing, 0.0)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw_offset)
            
            self.tf_broadcaster.sendTransform(
                relative_position,
                quaternion,
                rospy.Time.now(),
                "gps2",
                "utm"
            )
            rospy.loginfo("Published TF transform from 'utm' to 'gps' with relative position: {}, quaternion: {}".format(relative_position, quaternion))

if __name__ == '__main__':
    try:
        InitialPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

