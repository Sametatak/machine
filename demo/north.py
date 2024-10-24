#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseStamped
import math
import tf

def magnetic_field_callback(msg):
    # Extract magnetic field components
    mx = msg.magnetic_field.x
    my = msg.magnetic_field.y

    # Calculate heading in radians
    heading = math.atan2(my, mx)

    # Convert to degrees
    heading_deg = math.degrees(heading)

    # Correct for declination (example declination: 7 degrees)
    declination_deg = 7
    true_heading_deg = heading_deg + declination_deg

    # Normalize to 0-360 degrees
    if true_heading_deg < 0:
        true_heading_deg += 360
    elif true_heading_deg >= 360:
        true_heading_deg -= 360

    rospy.loginfo(f"Heading: {true_heading_deg:.2f} degrees")

    # Create PoseStamped message for RViz visualization
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "base_link"  # Base frame of your vehicle

    # Set position (keep at origin, only interested in orientation)
    pose_msg.pose.position.x = 0
    pose_msg.pose.position.y = 0
    pose_msg.pose.position.z = 0

    # Convert heading to quaternion for orientation
    quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(true_heading_deg))
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]

    # Publish the pose message
    pose_publisher.publish(pose_msg)

def magnetic_field_listener():
    global pose_publisher
    rospy.init_node('magnetic_field_heading', anonymous=True)
    pose_publisher = rospy.Publisher('/north_direction', PoseStamped, queue_size=10)
    rospy.Subscriber("/imu_bosch/mag", MagneticField, magnetic_field_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        magnetic_field_listener()
    except rospy.ROSInterruptException:
        pass

