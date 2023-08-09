#include "px4_interface/PX4_realsense_bridge.h"

using namespace bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_odom_interface_node");
  ros::NodeHandle nh("~");
  PX4_Realsense_Bridge Bridge(nh);
  ros::Rate rate(40);

  while (ros::ok()) {
  	Bridge.publishSystemStatus();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
