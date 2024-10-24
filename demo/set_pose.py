#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class SimpleAMCLNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('simple_amcl', anonymous=True)

        # Set up the TransformBroadcaster to publish the map to odom transform
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscribe to the "initialpose" topic to get the manually set pose
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.set_pose_callback)

        # Variable to store the current pose
        self.current_pose = None

    def set_pose_callback(self, msg):
        # Update the current pose with the received pose
        rospy.loginfo("Received new initial pose")
        self.current_pose = msg.pose.pose

    def publish_transform(self):
        if self.current_pose is not None:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = self.current_pose.position.x
            t.transform.translation.y = self.current_pose.position.y
            t.transform.translation.z = self.current_pose.position.z
            t.transform.rotation = self.current_pose.orientation

            # Broadcast the transform from map to odom
            self.tf_broadcaster.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                                              (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                                              t.header.stamp,
                                              t.child_frame_id,
                                              t.header.frame_id)

    def spin(self):
        rate = rospy.Rate(100)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            # Publish the transform from map to odom
            self.publish_transform()
            rate.sleep()

if __name__ == '__main__':
    try:
        simple_amcl = SimpleAMCLNode()
        simple_amcl.spin()
    except rospy.ROSInterruptException:
        pass

