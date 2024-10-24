#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import utm

def main():
    rospy.init_node('gps_goal_sender')

    listener = tf.TransformListener()

    # Publisher for the goal
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # Ensure the publisher connection is established
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        try:
            # Ask user for GPS coordinates
            lat_input = input("Enter latitude (or 'q' to quit): ").strip()
            if lat_input.lower() == 'q':
                rospy.loginfo("Exiting GPS goal sender.")
                break
            lat = float(lat_input)

            lon_input = input("Enter longitude: ").strip()
            lon = float(lon_input)
        except ValueError:
            rospy.logerr("Invalid input. Please enter numeric values for latitude and longitude.")
            continue

        # Ask user for the frame to publish the goal in
        frame_choice = input("Enter the frame to publish the goal in (utm/map/odom): ").strip().lower()
        if frame_choice not in ['utm', 'map', 'odom']:
            rospy.logerr("Invalid frame choice. Please enter 'utm', 'map', or 'odom'.")
            continue

        # Convert GPS to UTM
        utm_goal = utm.from_latlon(lat, lon)
        goal_easting = utm_goal[0]
        goal_northing = utm_goal[1]
        zone_number = utm_goal[2]
        zone_letter = utm_goal[3]

        rospy.loginfo("Goal UTM Coordinates: Easting: {}, Northing: {}, Zone Number: {}, Zone Letter: {}".format(
            goal_easting, goal_northing, zone_number, zone_letter))

        if frame_choice == 'utm':
            # Publish the goal directly in the UTM frame
            goal = PoseStamped()
            goal.header.frame_id = "utm"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = goal_easting
            goal.pose.position.y = goal_northing
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

        else:
            # Wait for transform from the chosen frame to UTM
            try:
                listener.waitForTransform("utm", frame_choice, rospy.Time(0), rospy.Duration(4.0))
            except tf.Exception as e:
                rospy.logerr("Transform from '{}' to 'utm' not available: {}".format(frame_choice, e))
                continue

            # Get robot's current position in UTM coordinates
            try:
                # Create a PoseStamped at the origin of the chosen frame
                frame_origin = PoseStamped()
                frame_origin.header.frame_id = frame_choice
                frame_origin.header.stamp = rospy.Time(0)
                frame_origin.pose.position.x = 0.0
                frame_origin.pose.position.y = 0.0
                frame_origin.pose.position.z = 0.0
                frame_origin.pose.orientation.x = 0.0
                frame_origin.pose.orientation.y = 0.0
                frame_origin.pose.orientation.z = 0.0
                frame_origin.pose.orientation.w = 1.0

                # Transform frame origin to UTM frame
                robot_in_utm = listener.transformPose("utm", frame_origin)

                robot_easting = robot_in_utm.pose.position.x
                robot_northing = robot_in_utm.pose.position.y

                rospy.loginfo("Robot UTM Coordinates: Easting: {}, Northing: {}".format(
                    robot_easting, robot_northing))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("Error transforming robot position to UTM frame: {}".format(e))
                continue

            # Calculate difference between goal and robot positions in UTM coordinates
            delta_easting = goal_easting - robot_easting
            delta_northing = goal_northing - robot_northing

            rospy.loginfo("Delta UTM Coordinates: Easting: {}, Northing: {}".format(
                delta_easting, delta_northing))

            # Create goal PoseStamped in UTM frame at the delta position
            delta_pose_utm = PoseStamped()
            delta_pose_utm.header.frame_id = "utm"
            delta_pose_utm.header.stamp = rospy.Time(0)
            delta_pose_utm.pose.position.x = robot_easting + delta_easting
            delta_pose_utm.pose.position.y = robot_northing + delta_northing
            delta_pose_utm.pose.position.z = 0.0
            delta_pose_utm.pose.orientation.x = 0.0
            delta_pose_utm.pose.orientation.y = 0.0
            delta_pose_utm.pose.orientation.z = 0.0
            delta_pose_utm.pose.orientation.w = 1.0

            # Transform the delta position to the chosen frame
            try:
                goal_in_frame = listener.transformPose(frame_choice, delta_pose_utm)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("Error transforming goal position to '{}' frame: {}".format(frame_choice, e))
                continue

            # Prepare the goal message
            goal = PoseStamped()
            goal.header.frame_id = frame_choice
            goal.header.stamp = rospy.Time.now()
            goal.pose = goal_in_frame.pose

        # Publish the goal
        pub.publish(goal)

        rospy.loginfo("Published goal to /move_base_simple/goal in '{}' frame".format(goal.header.frame_id))

        # Optionally, wait a moment before asking again
        rospy.sleep(0.5)

    rospy.loginfo("GPS goal sender node has been shut down.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
