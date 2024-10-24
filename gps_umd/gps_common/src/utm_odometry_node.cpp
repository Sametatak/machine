/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 * İlk GPS fix verisini referans alarak, sonraki verileri lokal koordinatlarda yayınlar
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;

// Referans noktayı saklamak için global değişkenler
double reference_easting = 0.0;
double reference_northing = 0.0;
double reference_altitude = 0.0; // Opsiyonel
bool initial_fix_received = false;

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_DEBUG_THROTTLE(60, "No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  // GPS koordinatlarını UTM koordinatlarına dönüştür
  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  // İlk GPS fix verisini referans olarak sakla
  if (!initial_fix_received) {
    reference_easting = easting;
    reference_northing = northing;
    reference_altitude = fix->altitude; // Opsiyonel
    initial_fix_received = true;
    ROS_INFO("Initial GPS fix received. Reference point set.");
  }

  // Lokal koordinatları hesapla
  double local_easting = easting - reference_easting;
  double local_northing = northing - reference_northing;
  double local_altitude = fix->altitude - reference_altitude; // Opsiyonel

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    // Frame ID ayarları
    if (frame_id.empty()) {
      if (append_zone) {
        odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = fix->header.frame_id;
      }
    } else {
      if (append_zone) {
        odom.header.frame_id = frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = frame_id;
      }
    }

    odom.child_frame_id = child_frame_id;

    // Odometri mesajını lokal koordinatlarla güncelle
    odom.pose.pose.position.x = local_easting;
    odom.pose.pose.position.y = local_northing;
    odom.pose.pose.position.z = 0 ;//local_altitude; // Eğer yükseklik farkını kullanmak istemezseniz, fix->altitude yazabilirsiniz

    // Oryantasyonu varsayılan değerlerle ayarla (GPS oryantasyon sağlamaz)
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Kovaryans matrisini oluştur
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0], fix->position_covariance[1], fix->position_covariance[2], 0, 0, 0,
      fix->position_covariance[3], fix->position_covariance[4], fix->position_covariance[5], 0, 0, 0,
      fix->position_covariance[6], fix->position_covariance[7], fix->position_covariance[8], 0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    // Odometri mesajını yayınla
    odom_pub.publish(odom);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  // Parametreleri yükle
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("append_zone", append_zone, false);

  // Odometri yayıncısını oluştur
  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  // GPS fix aboneliği oluştur
  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);

  // ROS döngüsünü başlat
  ros::spin();
}

