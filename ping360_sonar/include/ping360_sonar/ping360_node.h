#ifndef PING360_SONAR_NODE_H
#define PING360_SONAR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <ping360_msgs/msg/sonar_echo.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <ping360_sonar/sector.h>
#include <ping360_sonar/sonar_interface.h>

using namespace std::chrono_literals;
using ping360_msgs::msg::SonarEcho;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::Image;
using rcl_interfaces::msg::SetParametersResult;

namespace ping360_sonar
{

class Ping360Sonar : public rclcpp::Node
{
  using IntParams = std::map<std::string, int>;

public:
  Ping360Sonar(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  void refresh();

private:
  //Params
  std::string device_;
  int baudrate_;
  bool fallback_emulated_;
  std::string connection_type_;
  std::string udp_address_;
  int udp_port_;
  
  std::string frame_;
  int gain_;
  int frequency_ ;
  int range_max_ ;
  int angle_sector_ ;

  /// Custom mode
  bool custom_enabled_;
  int angle_min_;
  int angle_max_;
  /// Sector mode
  bool constrain_min_;
  int constrained_min_angle_;
  
  int angle_step_ ;
  int image_size_ ;
  int scan_threshold_ ;
  int speed_of_sound_ ;
  int image_rate_ ;
  int sonar_timeout_ ;
  
  bool publish_image_;
  bool publish_scan_;
  bool publish_echo_;
  bool publish_pcl_;

  rclcpp::TimerBase::SharedPtr image_timer;
  OnSetParametersCallbackHandle::SharedPtr param_change;
  SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  IntParams updatedParams(const std::vector<rclcpp::Parameter> &new_params) const;
  void configureFromParams(const std::vector<rclcpp::Parameter> &new_params = {});

  // helper functions to declare and describe parameters
  template <typename ParamType>
  inline ParamType declareParamDescription(std::string name,
                                           ParamType default_value,
                                           std::string description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    return declare_parameter<ParamType>(name, default_value, descriptor);
  }
  inline int declareParamDescription(std::string name,
                                     int default_value,
                                     std::string description,
                                     int lower,
                                     int upper,
                                     int step = 1)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                                .set__from_value(lower)
                                .set__to_value(upper)
                                .set__step(step)};
    return declare_parameter<int>(name, default_value, descriptor);
  }

  // sonar i/o
  std::shared_ptr<Ping360Interface> sonar;

  inline void initPublishers(bool image, bool scan, bool echo, bool pcl);

  // image params
  Sector sector;
  image_transport::Publisher image_pub;
  sensor_msgs::msg::Image image;
  void configureMessageFomParams();
  void refreshImage();
  inline void publishImage();

  // laserscan
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  LaserScan scan;
  int scan_threshold{};
  void publishScan(const rclcpp::Time &now, bool end_turn);

  // raw echo
  rclcpp::Publisher<ping360_msgs::msg::SonarEcho>::SharedPtr echo_pub;
  SonarEcho echo;
  void publishEcho(const rclcpp::Time &now);

  // raw pcl
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  PointCloud2 pcl;
  void publishPcl(const rclcpp::Time &now);

  std::vector<double> linspace(double start, double end, int num);
};
}

#endif