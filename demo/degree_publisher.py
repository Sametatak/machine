#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def publish_float_value(value):
    # Ensure the value is within the range -180 to 180
    if value < -180 or value > 180:
        rospy.logwarn("Value out of range. Please enter a value between -180 and 180.")
        return

    # Create a ROS publisher
    pub = rospy.Publisher('/float_value', Float32, queue_size=10)
    rospy.init_node('float_value_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Publish the value
    rospy.loginfo(f"Publishing value: {value}")
    pub.publish(value)
    rate.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # Get user input
            user_input = input("Enter a value between -180 and 180: ")

            try:
                float_value = float(user_input)
                publish_float_value(float_value)
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a numerical value.")
    except rospy.ROSInterruptException:
        pass

