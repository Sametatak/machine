#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
import utm

class StaticUTMTransform:
    def __init__(self):
        rospy.init_node('static_utm_tf_broadcaster')
        rospy.wait_for_message('/gps/fix', NavSatFix)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'utm')
        self.got_initial_fix = False

        self.br = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber('/gps/fix', NavSatFix, self.fix_callback)

        # Node'un kapanmasını engellemek için spin yerine rate kullanacağız
        self.rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()

    def fix_callback(self, msg):
        if not self.got_initial_fix:
            if msg.status.status == -1:  # STATUS_NO_FIX
                rospy.logwarn("Geçerli bir GPS fix alınamadı.")
                return

            # Enlem ve boylamı UTM koordinatlarına dönüştür
            try:
                easting, northing, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
            except Exception as e:
                rospy.logerr("UTM dönüşümü başarısız oldu: %s", e)
                return

            # TransformStamped mesajını oluştur
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id

            # UTM koordinatlarını transform olarak ayarla
            t.transform.translation.x = easting
            t.transform.translation.y = northing
            t.transform.translation.z = msg.altitude

            # Rotasyonu birim kuaterniyon olarak ayarla
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Statik transformu yayınla
            self.br.sendTransform(t)

            rospy.loginfo("Statik dönüşüm yayınlandı: %s -> %s", self.frame_id, self.child_frame_id)

            self.got_initial_fix = True

if __name__ == '__main__':
    try:
        StaticUTMTransform()
    except rospy.ROSInterruptException:
        pass

