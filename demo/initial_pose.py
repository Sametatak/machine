#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import utm
import math

class InitialPosePublisher:
    def __init__(self):
        rospy.init_node('initialpose_publisher', anonymous=True)
        
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.tf_listener = tf.TransformListener()
        
        self.orientation = None
        self.position = None
        self.yaw_offset = rospy.get_param('~yaw_offset', 1.57)  # Get yaw offset parameter

    def imu_callback(self, data):
        # Extract orientation from IMU
        self.orientation = data.orientation

    def gps_callback(self, data):
        # Extract position from GPS fix
        if not math.isnan(data.latitude) and not math.isnan(data.longitude):
            self.position = (data.latitude, data.longitude)
            
            # Convert to UTM coordinate system
            utm_goal = utm.from_latlon(self.position[0], self.position[1])
            self.position_utm = (utm_goal[0], utm_goal[1])
            
            self.calculate_pose()

    def calculate_pose(self):
        if self.position_utm and self.orientation:
            # Ask user for input to publish pose
            while not rospy.is_shutdown():
                user_input = input("Do you want to publish /initialpose? (yes/no): ").strip().lower()
                if user_input == 'yes':
                    try:
                        # Wait for transform from 'utm' to 'map'
                        self.tf_listener.waitForTransform('utm', 'map', rospy.Time(0), rospy.Duration(4.0))
                        
                        # Create a PoseStamped at the UTM position
                        utm_pose = PoseStamped()
                        utm_pose.header.frame_id = 'utm'
                        utm_pose.header.stamp = rospy.Time(0)
                        utm_pose.pose.position.x = self.position_utm[0]
                        utm_pose.pose.position.y = self.position_utm[1]
                        utm_pose.pose.position.z = 0.0

                        # Apply yaw offset to orientation
                        orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
                        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                        yaw += self.yaw_offset
                        new_orientation = quaternion_from_euler(roll, pitch, yaw)
                        utm_pose.pose.orientation.x = new_orientation[0]
                        utm_pose.pose.orientation.y = new_orientation[1]
                        utm_pose.pose.orientation.z = new_orientation[2]
                        utm_pose.pose.orientation.w = new_orientation[3]
                        
                        # Transform UTM pose to the 'map' frame
                        pose_in_map = self.tf_listener.transformPose('map', utm_pose)
                        
                        # Create PoseWithCovarianceStamped message
                        pose_msg = PoseWithCovarianceStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = 'map'
                        pose_msg.pose.pose = pose_in_map.pose
                        pose_msg.pose.covariance = [0.0] * 36
                        
                        # Publish /initialpose
                        self.pose_pub.publish(pose_msg)
                        rospy.loginfo("Published /initialpose with GPS and IMU data, including yaw offset")
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logerr("Error transforming UTM pose to 'map' frame: {}".format(e))
                elif user_input == 'no':
                    rospy.loginfo("Exiting initialpose publisher.")
                    break
                else:
                    rospy.logwarn("Invalid input. Please enter 'yes' or 'no'.")

if __name__ == '__main__':
    try:
        InitialPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        passs
