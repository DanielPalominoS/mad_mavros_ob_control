#ifndef LOCAL_WP_MANAGER_SERVER_H
#define LOCAL_WP_MANAGER_SERVER_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

//#include <mavros_offboard_msgs/.h>

#include <GeographicLib/Geodesic.hpp>
#include <boost/bind.hpp>

// Boost geometry libraries
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

//Mavros offboard msgs (own generated)
#include "mavros_offboard_msgs/CurrentWaypointInfo.h"
#include "mavros_offboard_msgs/LocalWaypoint.h"
#include "mavros_offboard_msgs/LocalWaypointList.h"
#include "mavros_offboard_msgs/MoveToNextWaypoint.h"
#include "mavros_offboard_msgs/SetWaypointThreshold.h"
#include "mavros_offboard_msgs/ModifyWaypointList.h"
#include "mavros_offboard_msgs/GetWaypointList.h"

namespace bg = boost::geometry;
// Definitions 
typedef bg::model::d2::point_xy<double> bg_xy_point;
typedef bg::model::point<double, 3, bg::cs::cartesian> bg_3d_point;
// Constants
static const std::string dflt_local_pose_sub_tp = "mavros/local_position/pose";
static const std::string dflt_local_state_sub_tp = "mavros/state";
static const std::string dflt_current_wp_pub_tp = "wp_manager/current_waypoint";

static const std::string dflt_modify_wp_list_srv_tp = "wp_manager/modify_wp_list";
static const std::string dflt_get_wp_list_srv_tp = "wp_manager/get_waypoint_list";
static const std::string dflt_move_to_next_wp_srv_tp = "wp_manager/move_to_next_waypoint";
static const std::string dflt_set_thr_srv_tp = "wp_manager/set_threshold";

static constexpr double dflt_distance_threshold = 1.0;
static constexpr double dflt_rate = 20.0; 

// Class
class waypoint_manager_server
{
  ros::NodeHandle nh_;
  // create messages that are used to published feedback/result
  // mavros_offboard_control::TakeoffRequest request_;
  // mavros_offboard_control::TakeoffResponse response_;
  
  //subscribers
  ros::Subscriber local_pose_subscriber_;  // subscriber to get the vehicle pose
  ros::Subscriber state_sub_;
  //publishers
  ros::Publisher current_wp_publisher_;  // publish the current wp info
  //Servers
  ros::ServiceServer modify_wp_list_server_;   // to modify the current wp_list
  ros::ServiceServer move_to_next_wp_server_;  // to move to nest wp in the wp_list
  ros::ServiceServer set_thr_server_;  // to move to nest wp in the wp_list
  ros::ServiceServer get_wp_list_server_;  // to move to nest wp in the wp_list

  //Attributes (variables)
  // ros::Time t_prev_local_;
  ros::Rate rate_;
  double prev_local_alt_;
  double curr_local_alt_;
  double goal_local_alt_;
  //double curr_yaw_;
  //double prev_yaw_; 
  double goal_yaw_;
  double pitch, roll;
  mavros_offboard_msgs::LocalWaypointList wp_list_;
  //mavros_offboard_msgs::LocalWaypoint curr_wp_;
  mavros_offboard_msgs::CurrentWaypointInfo curr_wp_info_;

  geometry_msgs::PoseStamped prev_local_pose_;
  geometry_msgs::PoseStamped curr_local_pose_;
  geometry_msgs::Quaternion curr_relative_attitude;
  geometry_msgs::Quaternion prev_relative_attitude;
  tf2::Quaternion goal_attitude_;

  double distance_to_goal_;
  double distance_threshold_xy_;
  double distance_threshold_z_;
  double yaw_threshold_;
  bool local_wp_reached_;
  //int reached_wp_index_;
  mavros_msgs::State current_state_;

public:
  waypoint_manager_server();
  ~waypoint_manager_server();
  void local_pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  /*
  computes relative between quaternions, stores the result in rel_attitude
  and returns the associated yaw
  */
  double compute_realtive_rotation(geometry_msgs::Quaternion attitude_1, geometry_msgs::Quaternion attitude_2, geometry_msgs::Quaternion* rel_attitude );
  

  bool set_thr_srv_cb(mavros_offboard_msgs::SetWaypointThreshold::Request &req,
                        mavros_offboard_msgs::SetWaypointThreshold::Response &res);

  bool modify_wp_list_srv_cb(mavros_offboard_msgs::ModifyWaypointList::Request &req,
                        mavros_offboard_msgs::ModifyWaypointList::Response &res);

  bool move_to_next_wp_srv_cb(mavros_offboard_msgs::MoveToNextWaypoint::Request &req,
                        mavros_offboard_msgs::MoveToNextWaypoint::Response &res);
  bool get_wp_list_srv_cb(mavros_offboard_msgs::GetWaypointList::Request &req,
                        mavros_offboard_msgs::GetWaypointList::Response &res);
  void state_cb(const mavros_msgs::State::ConstPtr &msg);

  void publish_wp_info();

  void upload_default_list();

  bool parse_file(std::string fila_name,mavros_offboard_msgs::LocalWaypointList* list);

  void run();
};

#endif  // LOCAL_WP_MANAGER_SERVER_H