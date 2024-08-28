#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geopy.distance import geodesic

def gps_to_xy(lat0, lon0, lat, lon):
    start = (lat0, lon0)
    end = (lat, lon)
    # Y eksenindeki mesafe (Kuzey-Güney)
    y = geodesic(start, (lat, lon0)).meters
    # X eksenindeki mesafe (Doğu-Batı)
    x = geodesic(start, (lat0, lon)).meters
    # Eğer enlem farkı negatife düşerse, y'yi negatif yap
    if lat < lat0:
        y = -y
    # Eğer boylam farkı negatife düşerse, x'i negatif yap
    if lon < lon0:
        x = -x
    return x, y

def create_marker(x, y, z=0):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    return marker

def create_goal(x, y):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    # Robotun başlangıçta kuzeye bakıyor olmasını varsayarak orientasyon ayarlanıyor
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal

def main():
    rospy.init_node('gps_goal_publisher')

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    # Başlangıç GPS koordinatları
    latitude_0 = 40.0000
    longitude_0 = 29.0000

    while not rospy.is_shutdown():
        try:
            # Kullanıcıdan hedef GPS koordinatlarını al
            latitude = float(input("Hedef Latitude (Enlem): "))
            longitude = float(input("Hedef Longitude (Boylam): "))
            
            # X-Y düzlemine çevir
            x, y = gps_to_xy(latitude_0, longitude_0, latitude, longitude)

            # Marker oluştur ve yayınla
            marker = create_marker(x, y)
            marker_pub.publish(marker)

            # Goal oluştur ve move_base'e yayınla
            goal = create_goal(x, y)
            goal_pub.publish(goal)

            print(f"Hedef başarıyla ayarlandı: X = {x}, Y = {y}")
            
        except Exception as e:
            print(f"Hata: {e}")
        rospy.sleep(1)

if __name__ == '__main__':
    main()
