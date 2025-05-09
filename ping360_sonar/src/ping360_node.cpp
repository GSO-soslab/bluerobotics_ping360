
#include <ping360_sonar/ping360_node.h>
#include <ping360_sonar/sector.h>
#include <ping-message-common.h>
#include <ping-message-ping360.h>

using namespace std::chrono_literals;
using namespace ping360_sonar;
using std::string;
using std::vector;

Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options)
  : Node("ping360_node", options)
{ 
  // Declare Params
  this->declare_parameter<std::string>("device");
  this->declare_parameter<int>("baudrate");
  this->declare_parameter<bool>("fallback_emulated");
  this->declare_parameter<std::string>("connection_type");
  this->declare_parameter<std::string>("udp_address");
  this->declare_parameter<int>("udp_port");

  this->declare_parameter<std::string>("frame");
  this->declare_parameter<int>("gain");
  this->declare_parameter<int>("frequency");
  this->declare_parameter<int>("range_max");
  this->declare_parameter<int>("angle_sector");

  this->declare_parameter<bool>("custom_enabled");
  this->declare_parameter<int>("angle_min");
  this->declare_parameter<int>("angle_max");
  this->declare_parameter<bool>("slice");
  this->declare_parameter<int>("min_angle");

  this->declare_parameter<int>("angle_step");
  this->declare_parameter<int>("image_size");
  this->declare_parameter<int>("scan_threshold");
  this->declare_parameter<int>("speed_of_sound");
  this->declare_parameter<int>("image_rate");
  this->declare_parameter<int>("sonar_timeout");

  //Publishers
  this->declare_parameter<bool>("publish_image");
  this->declare_parameter<bool>("publish_scan");
  this->declare_parameter<bool>("publish_echo");
  this->declare_parameter<bool>("publish_pcl");


  //Get Params
  device_ = this->get_parameter("device").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  fallback_emulated_ = this->get_parameter("fallback_emulated").as_bool();
  connection_type_ = this->get_parameter("connection_type").as_string();
  udp_address_ = this->get_parameter("udp_address").as_string();
  udp_port_ = this->get_parameter("udp_port").as_int();

  frame_ = this->get_parameter("frame").as_string();
  gain_ = this->get_parameter("gain").as_int();
  frequency_ = this->get_parameter("frequency").as_int();
  range_max_ = this->get_parameter("range_max").as_int();
  angle_sector_ = this->get_parameter("angle_sector").as_int();

  custom_enabled_ = this->get_parameter("custom_enabled").as_bool();
  angle_min_ = this->get_parameter("angle_min").as_int();
  angle_max_ = this->get_parameter("angle_max").as_int();
  slice_ = this->get_parameter("slice").as_bool();
  min_angle_ = this->get_parameter("min_angle").as_int();

  angle_step_ = this->get_parameter("angle_step").as_int();
  image_size_ = this->get_parameter("image_size").as_int();
  scan_threshold_ = this->get_parameter("scan_threshold").as_int();
  speed_of_sound_ = this->get_parameter("speed_of_sound").as_int();
  image_rate_ = this->get_parameter("image_rate").as_int();
  sonar_timeout_ = this->get_parameter("sonar_timeout").as_int();
  publish_image_ = this->get_parameter("publish_image").as_bool();
  publish_scan_ = this->get_parameter("publish_scan").as_bool();
  publish_echo_ = this->get_parameter("publish_echo").as_bool();
  publish_pcl_ = this->get_parameter("publish_pcl").as_bool();

  // Create Sonar Object
  sonar = std::make_shared<Ping360Interface>(device_, baudrate_, 
    fallback_emulated_, connection_type_, udp_address_, udp_port_);

  // constant initialization
  image.header.set__frame_id(frame_);
  image.set__encoding("mono8");
  image.set__is_bigendian(0);
  scan.header.set__frame_id(frame_);
  scan.set__range_min(0.75);
  echo.header.set__frame_id(frame_);

  // ROS interface
  configureFromParams();

  const auto image_rate_ms{image_rate_};
  image_timer = this->create_wall_timer(std::chrono::milliseconds(image_rate_ms),
                                        [this](){publishImage();});

  param_change = add_on_set_parameters_callback(
                   std::bind(&Ping360Sonar::parametersCallback, this, std::placeholders::_1));
}

Ping360Sonar::IntParams Ping360Sonar::updatedParams(const std::vector<rclcpp::Parameter> &new_params) const
{
  // "only" parameters to be monitored for change
  using ParamType = rclcpp::ParameterType;
  const std::map<ParamType,vector<string>> mutable_params{
    {ParamType::PARAMETER_INTEGER,{"gain","frequency","range_max",
                                   "angle_sector","angle_step",
                                   "speed_of_sound","image_size", "scan_threshold", "sonar_timeout"}},
    {ParamType::PARAMETER_BOOL, {"publish_image","publish_scan","publish_echo"}}};

  IntParams mapping;
  for(const auto &[type,names]: mutable_params)
  {
    const auto params{get_parameters(names)};
    if(type == ParamType::PARAMETER_INTEGER)
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_int();
    }
    else
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_bool();
    }
  }
  
  // override with new ones
  for(auto &param: new_params)
  {
    if(param.get_type() == ParamType::PARAMETER_BOOL)
      mapping[param.get_name()] = param.as_bool();
    else if(param.get_type() == ParamType::PARAMETER_INTEGER)
      mapping[param.get_name()] = param.as_int();
  }

  return mapping;
}

SetParametersResult Ping360Sonar::parametersCallback(const vector<rclcpp::Parameter> &parameters)
{
  configureFromParams(parameters);
  return SetParametersResult().set__successful(true);
}

void Ping360Sonar::initPublishers(bool image, bool scan, bool echo, bool pcl)
{
  if(image && image_pub.getTopic().empty())
    image_pub = image_transport::create_publisher(this, "msis/image");

  if(echo && echo_pub == nullptr)
    echo_pub = create_publisher<ping360_msgs::msg::SonarEcho>("msis/echo", 1);

  if(scan && scan_pub == nullptr)
    scan_pub = create_publisher<sensor_msgs::msg::LaserScan>("msis/scan", 1);

  if(pcl && pcl_pub == nullptr)
    pcl_pub = create_publisher<sensor_msgs::msg::PointCloud2>("msis/pointcloud", 1);
}

void Ping360Sonar::configureFromParams(const vector<rclcpp::Parameter> &new_params)
{
  // // get current params updated with new ones, if any
  // const auto params{updatedParams(new_params)};

  // forward to configuration
  const auto [angle_sector, step] = sonar->configureAngles(this->angle_sector_,
      this->angle_step_,
      this->publish_scan_,
      this->custom_enabled_,
      this->angle_min_,
      this->angle_max_); {}

      // inform if requested angle config cannot be met because of gradians
  if(angle_sector != this->angle_sector_ || step != this->angle_step_)
  {
    RCLCPP_INFO(get_logger(),
                "Due to sonar using gradians, sector is %i (requested %i) and step is %i (requested %i)",
                angle_sector, this->angle_sector_, step, this->angle_step_);
  }

  initPublishers(this->publish_image_,
                 this->publish_scan_,
                 this->publish_echo_,
                 this->publish_pcl_);

  sonar->configureTransducer(this->gain_,
                            this->frequency_,
                            this->speed_of_sound_,
                            this->range_max_);
  sonar->setTimeout(this->sonar_timeout_);

  // forward to message meta-data
  echo.set__gain(this->gain_);
  echo.set__range(this->range_max_);
  echo.set__speed_of_sound(this->speed_of_sound_);
  echo.set__number_of_samples(sonar->samples());
  echo.set__transmit_frequency(this->frequency_);

  scan.set__range_max(this->range_max_);
  scan.set__time_increment(sonar->transmitDuration());
  scan.set__angle_max(sonar->angleMax());
  scan.set__angle_min(sonar->angleMin());
  scan.set__angle_increment(sonar->angleStep());

  const int size{this->image_size_};
  if(size != static_cast<int>(image.step) ||
     std::any_of(new_params.begin(), new_params.end(),
                 [](const auto &param){return param.get_name() == "angle_sector";}))
  {
    image.data.resize(size*size);
    std::fill(image.data.begin(), image.data.end(), 0);
    image.height = image.width = image.step = size;
  }

  sector.configure(sonar->samples(), size/2);
  scan_threshold = this->scan_threshold_;

}


void Ping360Sonar::publishEcho(const rclcpp::Time &now)
{
  const auto [data, length] = sonar->intensities(); {}
  echo.angle = sonar->currentAngle();
  echo.intensities.resize(length);
  std::copy(data, data+length, echo.intensities.begin());
  echo.header.set__stamp(now);
  echo_pub->publish(echo);
}

void Ping360Sonar::publishScan(const rclcpp::Time &now, bool end_turn)
{
  // write latest reading
  scan.ranges.resize(sonar->angleCount());
  scan.intensities.resize(sonar->angleCount());

  const auto angle{sonar->angleIndex()};
  auto &this_range = scan.ranges[angle] = 0;
  auto &this_intensity = scan.intensities[angle] = 0;

  // find first (nearest) valid point in this direction
  const auto [data, length] = sonar->intensities(); {}
  for(int index=0; index<length; index++)
  {
    if(data[index] >= scan_threshold)
    {
      if(const auto range{sonar->rangeFrom(index)};
         range >= scan.range_min && range < scan.range_max)
      {
        this_range = range;
        this_intensity = data[index]/255.f;
        break;
      }
    }
  }

  if(end_turn)
  {
    if(!sonar->fullScan())
    {
      if(sonar->angleStep() < 0)
      {
        // now going negative: scan was positive
        scan.set__angle_max(sonar->angleMax());
        scan.set__angle_min(sonar->angleMin());
      }
      else
      {
        // now going positive: scan was negative
        scan.set__angle_max(sonar->angleMin());
        scan.set__angle_min(sonar->angleMax());
      }
      scan.set__angle_increment(-sonar->angleStep());
      scan.angle_max -= scan.angle_increment;
    }
    scan.header.set__stamp(now);
    scan_pub->publish(scan);
  }
}

void Ping360Sonar::refreshImage()
{
  const auto [data, length] = sonar->intensities(); {}
  if(length == 0) return;
  const auto half_size{image.step/2};

  sector.init(sonar->currentAngle(), fabs(sonar->angleStep()));
  int x{}, y{}, index{};

  while(sector.nextPoint(x, y, index))
  {
    if(index < length)
      image.data[half_size-y + image.step*(half_size-x)] = data[index];
  }
}

void Ping360Sonar::refresh()
{
  const auto &[valid, end_turn] = sonar->read(this->slice_, this->min_angle_); {}
  
  if(!valid)
  {
    RCLCPP_WARN(get_logger(), "Cannot communicate with sonar");
    return;
  }

  const auto now{this->now()};
  if(this->publish_echo_ && echo_pub->get_subscription_count())
    publishEcho(now);

  if(this->publish_image_)
    refreshImage();

  if(this->publish_scan_ && scan_pub->get_subscription_count())
    publishScan(now, end_turn);

  if(this->publish_pcl_ && pcl_pub->get_subscription_count())
  publishPcl(now);
}

void Ping360Sonar::publishImage()
{
  if(this->publish_image_)
  {
    image.header.set__stamp(now());
    image_pub.publish(image);
  }
}

void Ping360Sonar::publishPcl(const rclcpp::Time &now){
  //Get current intensity
    const auto [data, length] = sonar->intensities(); {}
  //Define message
    sensor_msgs::PointCloud2Modifier modifier(pcl);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    int number_of_bins;
    int range_min = 0.75;
    float cos_current_angle = std::cos(sonar->currentAngle());
    float sin_current_angle = std::sin(sonar->currentAngle());

    pcl.header.stamp = now;
    pcl.header.frame_id = this->frame_;
    pcl.height = 1;

  //Data sheet
    if (this->range_max_ == 1){
      number_of_bins = 666;
    }
    else{
      number_of_bins = 1200;
    }

    pcl.width = number_of_bins;
    pcl.is_dense = true;

    pcl.point_step = 16;
    pcl.row_step =pcl.point_step * pcl.width;;
    pcl.data.resize(pcl.width * pcl.point_step);

    std::vector<double> x = Ping360Sonar::linspace(range_min, this->range_max_, number_of_bins);

    sensor_msgs::PointCloud2Iterator<float> iterX(pcl, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl, "intensity");

    for (uint32_t i = 0; i < pcl.width; ++i) {
        *iterX = x[i] * cos_current_angle;
        *iterY = x[i] * sin_current_angle;
        *iterZ = 0;

        *iterIntensity = data[i];

        // // Increment the iterators
        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterIntensity;
    }
    pcl_pub->publish(pcl);
}

std::vector<double> Ping360Sonar::linspace(double start, double end, int num) {
  std::vector<double> result;
  double step = (end - start) / (num - 1);
  
  for (int i = 0; i < num; ++i) {
      double value = start + i * step;
      result.push_back(value);
  }
  
  return result;
}