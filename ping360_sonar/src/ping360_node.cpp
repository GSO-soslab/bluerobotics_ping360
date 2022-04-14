#include <ping360_sonar/ping360_ros.h> 

int main(int argc, char **argv) {
  ros::init(argc, argv, "ping360_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ping360_sonar::Ping360ROS  node(nh, nh_private);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    node.refresh();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}