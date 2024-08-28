#!/usr/bin/env python

import rospy
import utm
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def gps_to_utm(latitude, longitude):
    utm_coords = utm.from_latlon(latitude, longitude)
    return utm_coords[0], utm_coords[1]  # x (Eastings), y (Northings)

def publish_gps_points():
    # ROS düğümünü başlat
    rospy.init_node('gps_pointcloud_publisher', anonymous=True)
    
    # PointCloud mesajı için bir Publisher oluştur
    pointcloud_pub = rospy.Publisher('/gps_points', PointCloud, queue_size=10)
    
    # 2 farklı GPS noktası
    gps_coords = [
        (40.748817, -73.985428),  # New York, Empire State Building
        (48.858844, 2.294351)     # Paris, Eiffel Tower
    ]
    
    # GPS noktalarını UTM'ye dönüştürme
    utm_points = [Point32(x, y, 0.0) for x, y in [gps_to_utm(lat, lon) for lat, lon in gps_coords]]
    
    # PointCloud mesajını oluştur
    pointcloud_msg = PointCloud()
    pointcloud_msg.header.frame_id = "map"  # "map" frame kullanılıyor, duruma göre ayarlayın
    pointcloud_msg.header.stamp = rospy.Time.now()
    
    # UTM noktalarını mesajın points alanına ekle
    pointcloud_msg.points = utm_points
    
    # Mesajı yayınla
    rate = rospy.Rate(1)  # 1 Hz yayınlama hızı
    while not rospy.is_shutdown():
        pointcloud_msg.header.stamp = rospy.Time.now()  # Zaman damgasını güncelle
        pointcloud_pub.publish(pointcloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gps_points()
    except rospy.ROSInterruptException:
        pass

