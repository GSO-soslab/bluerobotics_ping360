#include <ping360_sonar/ping360_ros.h> 

#include "boost/date_time/posix_time/posix_time.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ping360_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ping360_sonar::Ping360ROS  node(nh, nh_private);

  int trigger_rate = node.getTriggerRate();
  ros::Rate loop_rate(trigger_rate);

  boost::posix_time::ptime t1, t2;
  while (ros::ok())
  {
    t1 = boost::posix_time::microsec_clock::local_time();
    node.refresh();
    t2 = boost::posix_time::microsec_clock::local_time();
    
    // printf("[TIME COST]: %.4f seconds\n", (t2-t1).total_microseconds() * 1e-6);

    ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
}