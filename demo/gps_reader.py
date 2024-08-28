#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import NavSatFix

def talker():
    pub = rospy.Publisher('gps_fix', NavSatFix, queue_size=10)
    rospy.init_node('gps_reader', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def filter_non_ascii(data):
        return ''.join([chr(c) if 32 <= c <= 126 else '' for c in data])
    
    while not rospy.is_shutdown():
        raw_line = ser.readline()
        line = filter_non_ascii(raw_line)
        if line.startswith('$GPGGA'):
            parts = line.split(',')
            if parts[2] and parts[4]:  # Valid data
                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "gps"
                msg.latitude = float(parts[2])
                msg.longitude = float(parts[4])
                pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
