#include "px4_interface/PX4_realsense_bridge.h"

namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh):nh_(nh) {
  odom_sub_topic = "/camera/odom/sample";
  world_frame = "odom";
  body_frame = "uav";
  nh_.getParam("cam_odom_topic", odom_sub_topic);
  nh_.getParam("world_frame", world_frame);
  nh_.getParam("body_frame", body_frame);

  tf_listener_ = new tf::TransformListener(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);
  odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>(odom_sub_topic, 1, &PX4_Realsense_Bridge::odomCallback, this);
  mavros_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
  mavros_system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);

  ROS_INFO("PX4_REALSENSE_BRIDGE Started.");
};

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() { delete tf_listener_; }

void PX4_Realsense_Bridge::odomCallback(const nav_msgs::Odometry& msg) {
  // publish odometry msg
  nav_msgs::Odometry output = msg;
  output.header.frame_id = world_frame;
  output.child_frame_id = body_frame;
  mavros_odom_pub_.publish(output);

  last_system_status_ = system_status_;
  // check confidence in vision estimate by looking at covariance
  if( msg.pose.covariance[0] > 0.2 ) // low confidence -> reboot companion
  {
    system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
  }
  else if( msg.pose.covariance[0] > 0.1 ) // medium confidence
  {
    system_status_ = MAV_STATE::MAV_STATE_CRITICAL;
  }
  else if( msg.pose.covariance[0] >= 0 ) // high confidence
  {
    system_status_ = MAV_STATE::MAV_STATE_ACTIVE;
  }
  else
  {
    ROS_WARN_STREAM("Unexpected vision sensor variance");
  }

}

void PX4_Realsense_Bridge::publishSystemStatus(){
  
  mavros_msgs::CompanionProcessStatus status_msg;

  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
  status_msg.state = (int)system_status_;

  mavros_system_status_pub_.publish(status_msg);
}

}
