#ifndef PING360_SONAR_ROS_H
#define PING360_SONAR_ROS_H

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ping360_msgs/SonarEcho.h>
#include <dynamic_reconfigure/server.h>

#include <ping360_sonar/sonar_interface.h>
#include <ping360_sonar/sector.h>
#include <ping360_sonar/SonarConfig.h>

namespace ping360_sonar
{

struct Params{
  /************ Serial interface ************/
  //// USB device name
  std::string device_;
  //// baudarate of serial device
  int baudrate_;
  //// Emulates a sonar if Ping360 cannot be initialized
  bool emulates_;

  std::string connection_type_; 

  std::string udp_address_;
  
  int udp_port_;

  /************ Sonar configuration ************/
  //// Sonar gain (0 = low, 1 = normal, 2 = high)
  int gain_;
  //// Sonar operating frequency [kHz], from 650 to 850
  int frequency_;
  //// Sonar max range [m]
  int range_;
  //// Scanned angular sector around sonar heading [degrees]
  int angle_sector_;
  /// If using custom angular sector [true/false]
  bool custom_sector_;
  /// Min angle.
  int angle_min_;
  /// Max angle.
  int angle_max_;
  //// Sonar angular resolution [degrees]
  int angle_step_;
  //// Speed of sound [m/s]
  int speed_of_sound_;
  //// trigger sonar to rotate [Hz]
  int trigger_rate_;

  /************ ROS Driver ************/
  //// frame_id in ROS messages
  std::string frame_id_;
  //// Publish images or not
  bool image_publish_;
  //// Output image size [pixels]
  int image_size_;
  //// Image publishing rate [ms]
  int image_rate_ms_;
  //// Publish scan or not
  bool scan_publish_;
  //// Intensity threshold for LaserScan message
  int scan_threshold_;
  //// Publish echo or not
  bool echo_publish_;
  //// Publish PointCloud or not
  bool pcl_publish_;
};

class Ping360ROS
{
public:
  Ping360ROS(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private);
  
  ~Ping360ROS() {}
  
  void loadParamters();
  
  void refresh();

  void serverCallback(ping360_sonar::SonarConfig &config, uint32_t level);

  int getTriggerRate() { return params.trigger_rate_; };

private:
  void configureFromParams();

  void publishEcho(const ros::Time &now);

  void publishScan(const ros::Time &now, bool end_turn);

  void publishImage(const ros::Time &now);

  void publishPointCloud2(const ros::Time &now);

  std::vector<double> linspace(double start, double end, int num);



  //// ros node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_config_;
  //// ros publishers
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_image_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_echo_;
  ros::Publisher pub_pcl_;
  //// ros dynamci reconfiguration
  dynamic_reconfigure::Server<ping360_sonar::SonarConfig> server_;

  //// sonar interface objects
  std::shared_ptr<Ping360Interface> sonar_;
  std::shared_ptr<Sector> sector_;

  //// sensor data
  sensor_msgs::Image image_;
  sensor_msgs::LaserScan scan_;
  ping360_msgs::SonarEcho echo_;
  sensor_msgs::PointCloud2 pcl_;

  //// paramters
  Params params;
};
  

}

#endif //PING360_SONAR_ROS_H
