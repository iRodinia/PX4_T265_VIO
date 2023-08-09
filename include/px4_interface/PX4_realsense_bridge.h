#ifndef PX4_REALSENSE_BRIDGE
#define PX4_REALSENSE_BRIDGE

#include <cstring>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace bridge {

  enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class PX4_Realsense_Bridge {
 public:
  PX4_Realsense_Bridge(const ros::NodeHandle& nh);
  ~PX4_Realsense_Bridge();

  void publishSystemStatus();

 private:
  ros::NodeHandle nh_;
  std::string odom_sub_topic, world_frame, body_frame;

  tf::TransformListener* tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Subscribers
  ros::Subscriber odom_sub_;
  // Publishers
  ros::Publisher mavros_odom_pub_;
  ros::Publisher mavros_system_status_pub_;

  MAV_STATE system_status_{MAV_STATE::MAV_STATE_UNINIT};
  MAV_STATE last_system_status_{MAV_STATE::MAV_STATE_UNINIT};

  void odomCallback(const nav_msgs::Odometry& msg);

};
}
#endif  // PX4_REALSENSE_BRIDGE
