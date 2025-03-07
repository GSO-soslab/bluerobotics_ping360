#include <ping360_sonar/ping360_ros.h>

#include <message/ping-message-common.h>
#include <message/ping-message-ping360.h>

using namespace ping360_sonar;

Ping360ROS::Ping360ROS(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private)
  : nh_(nh), nh_private_(nh_private), it_(nh_), 
    nh_config_("~Configuration"), server_(nh_config_)
{
  //// setup parameters
  loadParamters();

  //// setup objects
  sonar_ = std::make_shared<Ping360Interface>(params.device_, params.baudrate_, params.emulates_, params.connection_type_, params.udp_address_, params.udp_port_);
  sector_ = std::make_shared<Sector>();

  //// setup constant initialization for sensor data 
  scan_.header.frame_id = params.frame_id_;
  scan_.range_min = 0.75;

  echo_.header.frame_id = params.frame_id_;

  image_.header.frame_id = params.frame_id_;
  image_.encoding = "mono8";
  image_.is_bigendian = 0;

  configureFromParams();

  //// load dynamic reconfiguration
  server_.setCallback(boost::bind(&Ping360ROS::serverCallback, this, _1, _2));
}

void Ping360ROS::loadParamters() {

  //// Get serial interface parameters
  // if(!nh_private_.getParam("Interface/device", params.device_)) {
  //     ROS_ERROR("No Serial Device is given, exit now!");
  //     exit(EXIT_FAILURE);
  // }

  nh_private_.getParam("Interface/device", params.device_);
  nh_private_.param<int>("Interface/baudrate", params.baudrate_, 115200);
  nh_private_.param<bool>("Interface/emulates", params.emulates_, true);

  nh_private_.param<std::string>("Interface/connection_type", params.connection_type_, "udp");
  nh_private_.param<std::string>("Interface/udp_address", params.udp_address_, "192.168.2.104");
  nh_private_.param<int>("Interface/udp_port", params.udp_port_, 12345);

  //// Get sonar configuration parameters
  nh_private_.param<int>("Configuration/gain", params.gain_, 0);
  nh_private_.param<int>("Configuration/frequency", params.frequency_, 740);
  nh_private_.param<int>("Configuration/range", params.range_, 2);
  nh_private_.param<int>("Configuration/angle_step", params.angle_step_, 1);
  nh_private_.param<int>("Configuration/speed_of_sound", params.speed_of_sound_, 1500);
  nh_private_.param<int>("Configuration/trigger_rate", params.trigger_rate_, 10);

  //Get sector parameters
  nh_private_.param<int>("Sector/legacy_mode/angle_sector", params.angle_sector_, 360);

  nh_private_.param<bool>("Sector/custom_mode/enabled", params.custom_enabled_,false);
  nh_private_.param<int>("Sector/custom_mode/angle_min", params.angle_min_,0);
  nh_private_.param<int>("Sector/custom_mode/angle_max", params.angle_max_,90);
  
  nh_private_.param<bool>("Sector/custom_mode/slice", params.slice_,false);
  // nh_private_.param<bool>("Configuration/slice", params.slice_, true);
  nh_private_.param<int>("Sector/custom_mode/min_angle", params.min_angle_, 90); 
  // nh_private_.param<int>("Configuration/angle_max", params.angle_max_,400);



  //// Get ROS driver parameters
  nh_private_.param<std::string>("Driver/frame_id", params.frame_id_, "ping360_link");
  nh_private_.param<bool>("Driver/image_publish", params.image_publish_, true);
  nh_private_.param<int>("Driver/image_size", params.image_size_, 300);
  nh_private_.param<int>("Driver/image_rate_ms", params.image_rate_ms_, 100);
  nh_private_.param<bool>("Driver/scan_publish", params.scan_publish_, false);
  nh_private_.param<int>("Driver/scan_threshold", params.scan_threshold_, 200);
  nh_private_.param<bool>("Driver/echo_publish", params.echo_publish_, false);
  nh_private_.param<bool>("Driver/pcl_publish", params.pcl_publish_, true);

  //// Info all the parameters
  // ROS_INFO("\nSerial Parameters:\n device:%s, baudrate:%d", 
  //   params.device_.c_str(), params.baudrate_);
  // ROS_INFO("\nSonar Parameters:\n gain:%d, frequency:%d, range:%d, angle_sector:%d,"
  //                             " angle_step:%d, speed_of_sound:%d, frame_id_:%s",
  //   params.gain_, params.frequency_, params.range_, params.angle_sector_, 
  //   params.angle_step_, params.speed_of_sound_, params.frame_id_.c_str());
  // ROS_INFO("\nDriver Parameters:\n image_publish_:%s, image_size:%d, image_rate_ms_:%d,"
  //                              " scan_publish_:%s, scan_threshold:%d, echo_publish:%s", 
  //   params.image_publish_ ? "yes" : "no", params.image_size_, params.image_rate_ms_,
  //   params.scan_publish_ ? "yes" : "no", params.scan_threshold_, params.echo_publish_ ? "yes" : "no");
}

void Ping360ROS::serverCallback(ping360_sonar::SonarConfig &config, uint32_t level)
{
  // ROS_INFO("Reconfigure: gain:%d, frequency:%d, range:%d, angle_sector:%d, angle_step:%d, speed_of_sound:%d", 
  //           config.gain,
  //           config.frequency,
  //           config.range,
  //           config.angle_sector,
  //           config.angle_step,
  //           config.speed_of_sound);

  bool reconfigure = false;

  //// update parameters from dynamic recondiguration
  if(params.gain_ != config.gain) {
    params.gain_ = config.gain;
    reconfigure = true;
    ROS_INFO("[%s]: gain changed to %d", ros::this_node::getName().c_str(), params.gain_);
  }
  if(params.frequency_ != config.frequency) {
    params.frequency_ = config.frequency;
    reconfigure = true;
    ROS_INFO("[%s]: frequency changed to %d", ros::this_node::getName().c_str(), params.frequency_);
  }
  if(params.range_ != config.range) {
    params.range_ = config.range;
    reconfigure = true;
    ROS_INFO("[%s]: range changed to %d", ros::this_node::getName().c_str(), params.range_);
  }
  if(params.angle_sector_ != config.angle_sector) {
    params.angle_sector_ = config.angle_sector;
    reconfigure = true;
    ROS_INFO("[%s]: angle_sector changed to %d", ros::this_node::getName().c_str(), params.angle_sector_);
  }
  if(params.angle_step_ != config.angle_step) {
    params.angle_step_ = config.angle_step;
    reconfigure = true;
    ROS_INFO("[%s]: angle_step changed to %d", ros::this_node::getName().c_str(), params.angle_step_);
  }
  if(params.speed_of_sound_ != config.speed_of_sound) {
    params.speed_of_sound_ = config.speed_of_sound;
    reconfigure = true;
    ROS_INFO("[%s]: speed_of_sound changed to %d", ros::this_node::getName().c_str(), params.speed_of_sound_);
  }

  //// configure system based updated parameters
  if(reconfigure)
    configureFromParams();
}

void Ping360ROS::configureFromParams() {

  //// set sonar angle_sector and step
  const auto [angle_sector, step] = sonar_->configureAngles(params.angle_sector_,
                                                            params.angle_step_,
                                                            params.scan_publish_, 
                                                            params.custom_enabled_,
                                                            params.angle_min_,
                                                            params.angle_max_); {}

  if(angle_sector != params.angle_sector_ || step != params.angle_step_) {
    ROS_WARN("Due to sonar using gradians, sector is %i (requested %i) and step is %i (requested %i)",
              angle_sector, params.angle_sector_, step, params.angle_step_);
  }

  //// set sonar transducer
  sonar_->configureTransducer(params.gain_, params.frequency_, params.speed_of_sound_, params.range_);

  //// set scan meta-data
  scan_.angle_min = sonar_->angleMin();
  scan_.angle_max = sonar_->angleMax();
  scan_.angle_increment = sonar_->angleStep();
  scan_.time_increment = sonar_->transmitDuration();
  scan_.range_max = params.range_;

  //// set echo meta-data
  echo_.gain = params.gain_;
  echo_.number_of_samples = sonar_->samples();
  echo_.transmit_frequency = params.frequency_;
  echo_.speed_of_sound = params.speed_of_sound_;
  echo_.range = params.range_;

  //// set image meta-data
  const int size = params.image_size_;
  if(size != static_cast<int>(image_.step)) {
    image_.data.resize(size*size);
    std::fill(image_.data.begin(), image_.data.end(), 0);
    image_.height = image_.width = image_.step = size;  
  }

  //// setup sector
  sector_->configure(sonar_->samples(), size/2);

  //// setup publishers
  if(params.image_publish_)
    pub_image_ = it_.advertise("image", 10);
  if(params.scan_publish_)
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
  if(params.echo_publish_)
    pub_echo_ = nh_.advertise<ping360_msgs::SonarEcho>("echo", 10);
  if(params.pcl_publish_)
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud",10);
}

void Ping360ROS::refresh() {

  const auto &[valid, end_turn] = sonar_->read(params.slice_, params.min_angle_); {}
  // std::cout<<params.min_angle_<<std::endl;

  if(!valid)
  {
    ROS_WARN("[%s]: Cannot communicate with sonar", ros::this_node::getName().c_str());
    return;
  }

  const auto now = ros::Time::now();
  if(params.echo_publish_ && pub_echo_.getNumSubscribers())
    publishEcho(now);

  if(params.scan_publish_ && pub_scan_.getNumSubscribers())
    publishScan(now, end_turn);

  if(params.image_publish_)
    publishImage(now);

  if(params.pcl_publish_)
    publishPointCloud2(now);
}                       

void Ping360ROS::publishEcho(const ros::Time &now) {
  const auto [data, length] = sonar_->intensities(); {}
  echo_.angle = sonar_->currentAngle();
  echo_.intensities.resize(length);
  std::copy(data, data+length, echo_.intensities.begin());
  echo_.header.stamp = now;
  pub_echo_.publish(echo_);
}

void Ping360ROS::publishScan(const ros::Time &now, bool end_turn) {
  // write latest reading
  scan_.ranges.resize(sonar_->angleCount());
  scan_.intensities.resize(sonar_->angleCount());

  const auto angle{sonar_->angleIndex()};
  auto &this_range = scan_.ranges[angle] = 0;
  auto &this_intensity = scan_.intensities[angle] = 0;

  //// TODO: why only find nearest vaild point?
  // find first (nearest) valid point in this direction
  const auto [data, length] = sonar_->intensities(); {}
  for(int index=0; index<length; index++)
  {
    if(data[index] >= params.scan_threshold_)
    {
      if(const auto range{sonar_->rangeFrom(index)};
         range >= scan_.range_min && range < scan_.range_max)
      {
        this_range = range;
        this_intensity = data[index]/255.f;
        break;
      }
    }
  }

  if(end_turn)
  {
    if(!sonar_->fullScan())
    {
      if(sonar_->angleStep() < 0)
      {
        // now going negative: scan was positive
        scan_.angle_max = sonar_->angleMax();
        scan_.angle_min = sonar_->angleMin();
      }
      else
      {
        // now going positive: scan was negative
        scan_.angle_max = sonar_->angleMin();
        scan_.angle_min = sonar_->angleMax();
      }
      scan_.angle_increment = -sonar_->angleStep();
      scan_.angle_max -= scan_.angle_increment;
    }
    scan_.header.stamp = now;
    pub_scan_.publish(scan_);
  }
}

void Ping360ROS::publishImage(const ros::Time &now) {
  const auto [data, length] = sonar_->intensities(); {}
  if(length == 0) return;
  const auto half_size{image_.step/2};

  sector_->init(sonar_->currentAngle(), fabs(sonar_->angleStep()));
  int x{}, y{}, index{};

  while(sector_->nextPoint(x, y, index)) {
    if(index < length)
      image_.data[half_size-y + image_.step*(half_size-x)] = data[index];
  }

  image_.header.stamp = now;
  pub_image_.publish(image_);
}

void Ping360ROS::publishPointCloud2(const ros::Time &now){
  //Get current intensity
    const auto [data, length] = sonar_->intensities(); {}
  //Define message
    sensor_msgs::PointCloud2Modifier modifier(pcl_);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);

    int number_of_bins;
    int range_min = 0.75;
    float cos_current_angle = std::cos(sonar_->currentAngle());
    float sin_current_angle = std::sin(sonar_->currentAngle());

    pcl_.header.stamp = now;
    pcl_.header.frame_id = params.frame_id_;
    pcl_.height = 1;

  //Data sheet
    if (params.range_ == 1){
      number_of_bins = 666;
    }
    else{
      number_of_bins = 1200;
    }

    pcl_.width = number_of_bins;
    pcl_.is_dense = true;

    pcl_.point_step = 16;
    pcl_.row_step =pcl_.point_step * pcl_.width;;
    pcl_.data.resize(pcl_.width * pcl_.point_step);

    std::vector<double> x = Ping360ROS::linspace(range_min, params.range_, number_of_bins);
    
    // for (const auto& number : x) {
    //     std::cout << number << " ";
    // }
    // std::cout << std::endl;
    
    // // Using a traditional for loop
    // for (size_t i = 0; i < x.size(); ++i) {
    //     std::cout << x[i] << " ";
    // }
    // std::cout << std::endl;

    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_, "intensity");

    for (int i = 0; i < pcl_.width; ++i) {
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
    pub_pcl_.publish(pcl_);
}

std::vector<double> Ping360ROS::linspace(double start, double end, int num) {
    std::vector<double> result;
    double step = (end - start) / (num - 1);
    
    for (int i = 0; i < num; ++i) {
        double value = start + i * step;
        result.push_back(value);
    }
    
    return result;
}
