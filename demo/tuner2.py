#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(data):
    # Change the frame_id of the map
    data.header.frame_id = "custom_map_frame"
    pub.publish(data)

rospy.init_node('map_frame_remap')
pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=10)
rospy.Subscriber('/map', OccupancyGrid, map_callback)
rospy.spin()

