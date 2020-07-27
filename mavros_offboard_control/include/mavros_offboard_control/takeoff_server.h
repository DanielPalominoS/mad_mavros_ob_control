
#ifndef TAKEOFF_SERVER_H
#define TAKEOFF_SERVER_H
#include <ros/ros.h>
//#include <ros/rate.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/NavSatFix.h>

#include <mavros_offboard_control/Takeoff.h>

#include <GeographicLib/Geodesic.hpp>

static const std::string kDefaultTakeoffServerTopic = "offboard_controller/takeoff_server/check_takeoff";
static const std::string kDefaultGpsSubTopic = "mavros/global_position/global";
static const std::string kDefaulLocalPoseSubTopic = "mavros/local_position/pose";
static constexpr bool kDefaultLocalFrameFlag=false;
static constexpr double kDefaultDistanceThreshold  =1.0;
static constexpr double kDefaultRate =1.0 ;

class takeoff_server
{
  ros::NodeHandle nh_;
  // create messages that are used to published feedback/result
  mavros_offboard_control::TakeoffRequest request_;
  //mavros_offboard_control::TakeoffResponse response_;

  ros::Subscriber Pose_Subscriber_;
  ros::Subscriber GpsSubscriber_;
  ros::ServiceServer  Takeoff_server_;

  ros::Time t_prev_local_;
  ros::Time t_prev_gps_;
  ros::Rate rate_;
  //sensor_msgs::NavSatFix CurrentPose_;
  double prev_alt_gps_;
  double prev_alt_local_;
  sensor_msgs::NavSatFix  prev_gps;
  sensor_msgs::NavSatFix  curr_gps;

  geometry_msgs::Pose prev_local_pose;
  geometry_msgs::Pose curr_local_pose;
  double curr_alt_gps_;
  double curr_alt_local_;

  double distance_to_goal_gps_;
  double distance_to_goal_local_;
  double distance_threshold_;

  bool  use_local_frame_;

  bool  success_gps_;
  bool  success_local_;
public:
  takeoff_server();
  ~takeoff_server();
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& msg);
  void PoseCallback(const geometry_msgs::PoseStampedPtr& msg);
  bool CheckTakeoff(mavros_offboard_control::Takeoff::Request &req,
                        mavros_offboard_control::Takeoff::Response &res);
  void Main_task();

};

#endif // TAKEOFF_SERVER_H
