#ifndef MAVROS_OFFBOARD_CONTROL_NODE_H
#define MAVROS_OFFBOARD_CONTROL_NODE_H
#include <ros/ros.h>
#include <ros/rate.h>
#include  <tf/tf.h>
//#include  <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include  <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>

#include <crop_image_processing/Crop.h>
#include <crop_image_processing/Image_point.h>
#include <crop_image_processing/Find_crop_features.h>
#include <crop_image_processing/Take_save_picture.h>

#include <mavros_offboard_control/Takeoff.h>
#include <mavros_offboard_control/Check_waypoint.h>

#include <flight_planning/generate_plan.h>

#include <GeographicLib/Geodesic.hpp>
//Default subscription topics
static const std::string kDefaultGpsSubTopic = "mavros/global_position/global";
static const std::string kDefaultLocalPoseSubTopic = "mavros/local_position/pose";
static const std::string kDefaultImuSubTopic = "mavros/imu/data";
static const std::string kDefaultHeadingSubTopic = "mavros/global_position/compass_hdg";
static const std::string kDefaultStateSubTopic = "mavros/state";

//static const std::string kDefaultSetModeSubTopic = "mavros/set_mode";
//Default publisher topics
static const std::string kDefaultSetpointGlobalPubTopic = "mavros/setpoint_position/global";
//static const std::string kDefaultSetpointLocalPubTopic = "mavros/setpoint_position/local";
//static const std::string kDefaultSetpointGlobalPubTopic = "mavros/setpoint_raw/global";
static const std::string kDefaultSetpointLocalPubTopic = "mavros/setpoint_raw/local";

//Default services topics
static const std::string kDefaultCmdArmingbTopic = "mavros/cmd/arming";
static const std::string kDefaultSetModeTopic = "mavros/set_mode";
static const std::string kDefaultTakePicTopic = "crop_image_server/take_picture";
static const std::string kDefaultProcessImgTopic = "crop_image_server/find_crop_features";
static const std::string kDefaultPlanFlightTopic = "flight_planner/plan_flight";
static const std::string kDefaultWpCtrlTopic = "offboard_controller/wp_server/check_wp";
static const std::string kDefaultTakeoffTopic = "offboard_controller/takeoff_server/check_takeoff";

static constexpr double kDefaultFreq =200.0 ;
static constexpr  double  kDefaultTakeoffHeight = 120.0;

static constexpr  double  kDefaultDesiredGsd = 0.04;
static constexpr  int  kDefaultDesiredEndlap = 70;
static constexpr  int  kDefaultDesiredSidelap = 60;
static constexpr  double  kDefaultCamOri = 90;

static constexpr  double  kDefaultImgWidth = 1280;
static constexpr  double  kDefaultImgHeight = 960;
static constexpr  double  kDefaultFx = 1464;
static constexpr  double  kDefaultFy = 1464;
static constexpr  double  kDefaultCx = 640;
static constexpr  double  kDefaultCy = 480;
static constexpr  double  kDefaultSkew = 0;
static constexpr  double  kDefaultFocalLenght = 0.0055;
static constexpr  double  kDefaultSensorWidth = 0.0048;
static constexpr  double  kDefaultSensorHeight = 0.0036;

class mavros_offboard_control_node
{
  ros::NodeHandle nh_;

  ros::Subscriber global_pos_sub_; 
  ros::Subscriber local_pos_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber state_sub_;

  ros::Publisher global_pos_pub_;
  ros::Publisher local_pos_pub_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient take_picture_client_;
  ros::ServiceClient process_image_client_;
  ros::ServiceClient plan_flight_client_;
  ros::ServiceClient wp_control_client_;
  ros::ServiceClient takeoff_client_;

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate_;

  mavros_msgs::CommandBool  arming_cmd_srv_;
  mavros_msgs::SetMode  set_mode_srv_;
  crop_image_processing::Take_save_picture  save_pict_srv_;
  crop_image_processing::Find_crop_features find_crop_srv_;
  flight_planning::generate_plan  generate_plan_srv_;
  mavros_offboard_control::Check_waypoint check_wp_srv_;
  mavros_offboard_control::Takeoff  takeoff_srv_;



  /******** Image variables ********/
  //buscar forma de almacenarlas en un archivo
  double  i_height_;
  double  i_width_;
  double  frame_rate_;
  double  fx_;
  double  fy_;
  double  cx_;
  double  cy_;
  double  skew_;
  double  focal_lenght_;
  double  s_width_;
  double  s_heigth_;

  /******** Photogrammetric variables   ********/
  //Buscar la forma de psarlas como parametros para ejecutar el nodo
  double  desired_gsd_;
  int  desired_sidelap_;
  int  desired_endlap_;
  double  cam_ori_;

  double  uav_speed_;

  /******** Local variables *********/
  //ros::Publisher  vel_publisher_;
  //Gps variables
  sensor_msgs::NavSatFix  current_gps_pose_;
  sensor_msgs::NavSatFix  initial_gps_pose_;
  sensor_msgs::NavSatFix  desired_pose_;

  geometry_msgs::PoseStamped  current_local_pose_;

  std_msgs::Float64  current_heding_;

  geometry_msgs::Vector3  current_gps_;
  //IMU variables
  sensor_msgs::Imu  uav_attitude_;
  geometry_msgs::Vector3  uav_ypr_;

  uint current_wp_index_;
  double  steering_angle_;

  double  takeoff_height_;

  /*******control flags*******/
  bool  start_flag_;
  bool  debug_flag_;
  bool  fcu_connected_;
  bool  takeoff_done_flag_;
  bool  plan_available_flag_;
  bool  crop_found_flag_;
  bool  are_degrees_flag_;
  bool  plan_executed_flag_;
  bool  photo_taken_flag_;  
  bool  waypoint_reached_;
  //bool  waypoint_available_;
  bool  error_ocurred_flag_;
  bool  timeout_exceeded_flag_;

  std_msgs::String  previous_mode_;

  void  gps_cb(const sensor_msgs::NavSatFixConstPtr& msg);
  void  local_pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void  imu_cb(const sensor_msgs::ImuConstPtr& msg);
  void  heading_cb(const  std_msgs::Float64ConstPtr& msg);
  void  state_cb(const mavros_msgs::State::ConstPtr& msg);

  void  setpoint_local_pub();
  void  setpoint_global_pub();

  bool  arming_uav(bool enable);
  bool  takeoff_uav(double takeoff_rel_height);

  void  update_generate_plan_request();


  double  timeout_;
public:
  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_offboard_control_node();
  ~mavros_offboard_control_node();
  void  main_task();
  double  calculate_height(double fl,double gsd,double sw,double iw);
  double  calculate_gsd(double fl,double height,double sw,double iw);
  bool  wait_for_servers();
  //bool  waypoint_cb();


};

#endif // MAVROS_OFFBOARD_CONTROL_NODE_H
