#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class GPSNavigator:
    def __init__(self, target_lat, target_lon):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.current_lat = None
        self.current_lon = None
        self.velocity_publisher = rospy.Publisher('/move_base/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
    def gps_callback(self, data):
        self.current_lat = data.latitude
        self.current_lon = data.longitude
        self.navigate()

    def navigate(self):
        if self.current_lat is None or self.current_lon is None:
            return
        
        # Hedefe olan açıyı hesapla
        angle_to_target = self.calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        
        # Açıyı kullanarak bir hareket mesajı oluştur
        twist_msg = Twist()
        twist_msg.linear.x = 1 # İleri hareket hızı
        twist_msg.angular.z = angle_to_target
        if angle_to_target > 1:
            twist_msg.linear.x = 0
        
        # Mesajı yayınla
        self.velocity_publisher.publish(twist_msg)
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Radyan cinsine çevir
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        
        d_lon = lon2 - lon1
        
        y = math.sin(d_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
        bearing = math.atan2(y, x)
        
        return bearing

if __name__ == '__main__':
    try:
        rospy.init_node('gps_navigator')
        
        # Kullanıcıdan hedef Latitude ve Longitude değerlerini al
        target_lat = float(input("Hedef Latitude: "))
        target_lon = float(input("Hedef Longitude: "))
        
        navigator = GPSNavigator(target_lat, target_lon)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
