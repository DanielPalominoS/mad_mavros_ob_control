#include "mavros_offboard_control/local_wp_manager.h"
/*************
 * Constructor
 *************/
waypoint_manager_server::waypoint_manager_server() : rate_(dflt_rate)
{
  ros::NodeHandle pnh("~");
  // subscribers
  local_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      dflt_local_pose_sub_tp, 5, boost::bind(&waypoint_manager_server::local_pose_cb, this, _1));
  state_sub_ = nh_.subscribe<mavros_msgs::State>(dflt_local_state_sub_tp, 5,
                                                 boost::bind(&waypoint_manager_server::state_cb, this, _1));
  // publishers
  current_wp_publisher_ = nh_.advertise<mavros_offboard_msgs::CurrentWaypointInfo>(dflt_current_wp_pub_tp, 10);

  // Servers

  modify_wp_list_server_ =
      nh_.advertiseService(dflt_modify_wp_list_srv_tp, &waypoint_manager_server::modify_wp_list_srv_cb, this);

  move_to_next_wp_server_ =
      nh_.advertiseService(dflt_move_to_next_wp_srv_tp, &waypoint_manager_server::move_to_next_wp_srv_cb, this);

  set_thr_server_ = nh_.advertiseService(dflt_set_thr_srv_tp, &waypoint_manager_server::set_thr_srv_cb, this);

  get_wp_list_server_ =
      nh_.advertiseService(dflt_get_wp_list_srv_tp, &waypoint_manager_server::get_wp_list_srv_cb, this);
  // variables

  distance_threshold_xy_ = 0.02;
  distance_threshold_z_ = 2.0 * distance_threshold_xy_;
  yaw_threshold_ = 5.0 * M_PI / 180.0;

  prev_local_alt_ = 0;
  prev_local_pose_.pose.position.x = 0.0;
  prev_local_pose_.pose.position.y = 0.0;
  prev_local_pose_.pose.position.z = 0.0;
  prev_local_pose_.pose.orientation.x = 0.0;
  prev_local_pose_.pose.orientation.y = 0.0;
  prev_local_pose_.pose.orientation.z = 0.0;
  prev_local_pose_.pose.orientation.w = 1.0;
  prev_relative_attitude.x = 0.0;
  prev_relative_attitude.y = 0.0;
  prev_relative_attitude.z = 0.0;
  prev_relative_attitude.w = 1.0;

  curr_local_alt_ = 0;
  curr_local_pose_.pose.position.x = 0.0;
  curr_local_pose_.pose.position.y = 0.0;
  curr_local_pose_.pose.position.z = 0.0;
  curr_local_pose_.pose.orientation.x = 0.0;
  curr_local_pose_.pose.orientation.y = 0.0;
  curr_local_pose_.pose.orientation.z = 0.0;
  curr_local_pose_.pose.orientation.w = 1.0;
  curr_relative_attitude.x = 0.0;
  curr_relative_attitude.y = 0.0;
  curr_relative_attitude.z = 0.0;  // 0.7071068;
  curr_relative_attitude.w = 1.0;
  // 0.7071068;

  goal_local_alt_ = 5.0 * distance_threshold_z_;
  goal_yaw_ = M_PI_2;
  // goal_attitude_.setRPY(0, 0, M_PI_2);
  mavros_offboard_msgs::LocalWaypoint lwp;
  /*lwp.heading = M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0.5;
  wp_list_.current_seq = 0;
  wp_list_.local_waypoints.push_back(lwp);*/
  this->upload_default_list();

  local_wp_reached_ = false;
  curr_wp_info_.reached = false;
  curr_wp_info_.total_wp = 0;
  curr_wp_info_.wp_index = 0;
  curr_wp_info_.current_wp.heading = goal_yaw_;
  curr_wp_info_.current_wp.local_waypoint.x = 2.0 * distance_threshold_xy_;
  curr_wp_info_.current_wp.local_waypoint.y = 2.0 * distance_threshold_xy_;
  curr_wp_info_.current_wp.local_waypoint.z = 2.0 * distance_threshold_z_;
}

/*************
 * Destructor
 *************/
waypoint_manager_server::~waypoint_manager_server()
{
}

/*************
 * Callback for loscal pose subscriber
 *************/
void waypoint_manager_server::local_pose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  /*A time gap between the current and previous
  poses to be compared is considered, so that the vehicle would be more likely to reach and have a stable pose at the
  wp*/
  /*#ifdef DEBUG
    ROS_INFO("diff time: ");
    std::cout << (msg->header.stamp - curr_local_pose_.header.stamp).toSec() << std::endl;
  #endif*/
  if (fabs((msg->header.stamp - curr_local_pose_.header.stamp).toSec()) > 0)
  {
    // if((ros::Time::now() - ros::Duration(1.0)).sec>0){
    /*#ifdef DEBUG
        ROS_INFO("prev updated");
    #endif*/
    prev_local_pose_ = curr_local_pose_;
  }
  curr_local_pose_ = *msg;
}
/*************
 * Callback for loscal pose subscriber
 *************/
void waypoint_manager_server::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
  return;
}

/*************
 * Helper to compute relative rotation between two poses
 *
 * Takes as argument two attitudes (attitude1,attitude2), then calculate the relative orientation and stores the
 *result(rel_attitude). All attitude are expressed as quaternions Returns the yaw difference
 *************/
double waypoint_manager_server::compute_realtive_rotation(geometry_msgs::Quaternion attitude_1,
                                                          geometry_msgs::Quaternion attitude_2,
                                                          geometry_msgs::Quaternion *rel_attitude)
{
  // tf2::Quaternion q1(attitude_1.x, attitude_1.y, attitude_1.z, attitude_1.w);
  double roll, pitch, yaw;
  tf2::Quaternion q1, q2, qr;
  tf2::convert(attitude_1, q1);
  tf2::convert(attitude_2, q2);
  // q1.inverse();
  qr = q2 * -q1.inverse();
  tf2::Matrix3x3 rm(qr);
  rm.getRPY(roll, pitch, yaw);
  *rel_attitude = tf2::toMsg(qr);
  return yaw;
}
/*************
 * Callback for the service in charge of setting the thresholds
 *
 * Takes as argument the corresponding request and response objects
 * associated with the service
 * Returns true if the transaction is successful
 *************/
bool waypoint_manager_server::set_thr_srv_cb(mavros_offboard_msgs::SetWaypointThreshold::Request &req,
                                             mavros_offboard_msgs::SetWaypointThreshold::Response &res)
{
  bool success = false;
  distance_threshold_z_ = req.thr_z;
  distance_threshold_xy_ = req.thr_xy;
  yaw_threshold_ = req.thr_yaw;
  success = true;
  res.success = success;
  return success;
}
/*************
 * Callback for the service in charge of modifying the list of waypoints
 *
 * Takes as argument the corresponding request and response objects
 * associated with the service
 * Returns true if the transaction is successful
 *************/
bool waypoint_manager_server::modify_wp_list_srv_cb(mavros_offboard_msgs::ModifyWaypointList::Request &req,
                                                    mavros_offboard_msgs::ModifyWaypointList::Response &res)
{
  bool success = false;
  wp_list_ = req.new_wp_list;
  curr_wp_info_.mission_completed = false;
  this->publish_wp_info();

  success = true;
  res.success = success;
  return success;
}
/*************
 * Callback for the service in charge ofmoving to the next waypoint
 *
 * Takes as argument the corresponding request and rseponse objects
 * associated with the service
 * Returns true if the transaction is successful
 *************/
bool waypoint_manager_server::move_to_next_wp_srv_cb(mavros_offboard_msgs::MoveToNextWaypoint::Request &req,
                                                     mavros_offboard_msgs::MoveToNextWaypoint::Response &res)

{
  bool success = false;

  if (wp_list_.current_seq < wp_list_.local_waypoints.size() - 1)
  {
    wp_list_.current_seq++;
    curr_wp_info_.mission_completed = false;
  }
  else if (wp_list_.current_seq == wp_list_.local_waypoints.size() - 1)
  {
    curr_wp_info_.wp_index = curr_wp_info_.total_wp - 1;
    curr_wp_info_.mission_completed = true;
#ifdef DEBUG
    ROS_INFO_ONCE("Mission completed");
#endif
  }
  success = true;
  // response success set to false when there is no wp available to move to
  res.success = success;
  local_wp_reached_ = false;
  this->publish_wp_info();
  ros::spinOnce();
  rate_.sleep();

  return true;
}
/*************
 * Callback for the service in charge of provide the waypointlist on request
 *
 * Takes as argument the corresponding request and rseponse objects
 * associated with the service
 * Returns true if the transaction is successful
 *************/
bool waypoint_manager_server::get_wp_list_srv_cb(mavros_offboard_msgs::GetWaypointList::Request &req,
                                                 mavros_offboard_msgs::GetWaypointList::Response &res)
{
  bool success = false;
  res.local_wp_list = wp_list_;
  success = true;
  res.success = success;
  return success;
}

void waypoint_manager_server::upload_default_list()
{
  mavros_offboard_msgs::LocalWaypoint lwp;
  wp_list_.current_seq = 0;
  lwp.heading = 1*M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0.3;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading = 1 * M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 1;
  lwp.local_waypoint.z = 0.3;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading = 0 * M_PI_2;
  lwp.local_waypoint.x = 0.5;
  lwp.local_waypoint.y = 1;//1.5;
  lwp.local_waypoint.z = 0.3;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading = 1 * M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0; //1.5;
  lwp.local_waypoint.z = 0.3;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading = 2 * M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0.3;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading = 1 * M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0;
  wp_list_.local_waypoints.push_back(lwp);
  
  /*lwp.heading = 3 * M_PI_2;
  lwp.local_waypoint.x = 0.75;
  lwp.local_waypoint.y = -1;
  lwp.local_waypoint.z = 0.4;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading =  M_PI;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = -1;
  lwp.local_waypoint.z = 0.4;
  wp_list_.local_waypoints.push_back(lwp);

  lwp.heading =  M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0.4;
  wp_list_.local_waypoints.push_back(lwp);
  
  lwp.heading = 0 * M_PI_2;
  lwp.local_waypoint.x = 0;
  lwp.local_waypoint.y = 0;
  lwp.local_waypoint.z = 0.4;
  wp_list_.local_waypoints.push_back(lwp);*/

}

void waypoint_manager_server::publish_wp_info()
{
  curr_wp_info_.current_wp = wp_list_.local_waypoints[wp_list_.current_seq];
  curr_wp_info_.reached = local_wp_reached_;
  curr_wp_info_.total_wp = wp_list_.local_waypoints.size();
  curr_wp_info_.wp_index = wp_list_.current_seq;

  current_wp_publisher_.publish(curr_wp_info_);
}

/*************
 * primary function
 *
 * This fucntion intiates the execution of publishers
 * and updates the information peridiocally
 *************/
void waypoint_manager_server::run()
{
  // wait for FCU connection
  while (ros::ok() && !current_state_.connected)
  {
    ros::spinOnce();
    rate_.sleep();
  }
#ifdef DEBUG
  ROS_INFO_ONCE("Fcu connected");
#endif
  while (ros::ok() && current_state_.connected)
  {
    bg_xy_point curr_p = bg_xy_point(curr_local_pose_.pose.position.x, curr_local_pose_.pose.position.y);
    bg_xy_point prev_p = bg_xy_point(prev_local_pose_.pose.position.x, prev_local_pose_.pose.position.y);
    bg_xy_point goal_p = bg_xy_point(wp_list_.local_waypoints[wp_list_.current_seq].local_waypoint.x,
                                     wp_list_.local_waypoints[wp_list_.current_seq].local_waypoint.y);

    curr_local_alt_ = curr_local_pose_.pose.position.z;
    prev_local_alt_ = curr_local_pose_.pose.position.z;
    goal_local_alt_ = wp_list_.local_waypoints[wp_list_.current_seq].local_waypoint.z;
    goal_yaw_ = wp_list_.local_waypoints[wp_list_.current_seq].heading;
    ROS_INFO("Goal attitude");
    goal_attitude_.setRPY(0, 0, goal_yaw_);
    std::cout << tf2::toMsg(goal_attitude_) << std::endl;

    ROS_INFO("Current attitude");
    std::cout << curr_local_pose_.pose.orientation << std::endl;

    double curr_heading_diff = this->compute_realtive_rotation(curr_local_pose_.pose.orientation,
                                                               tf2::toMsg(goal_attitude_), &curr_relative_attitude);
    double prev_heading_diff = this->compute_realtive_rotation(prev_local_pose_.pose.orientation,
                                                               tf2::toMsg(goal_attitude_), &prev_relative_attitude);
    ROS_INFO("Relative attitude");
    std::cout << prev_relative_attitude << std::endl;
    std::cout << curr_relative_attitude << std::endl;

    if (bg::distance(curr_p, goal_p) < distance_threshold_xy_ &&
        bg::distance(prev_p, goal_p) < distance_threshold_xy_ &&
        fabs(goal_local_alt_ - curr_local_alt_) < distance_threshold_z_ &&
        fabs(goal_local_alt_ - prev_local_alt_) < distance_threshold_z_ && fabs(curr_heading_diff) < yaw_threshold_ &&
        fabs(prev_heading_diff) < yaw_threshold_)
    {
      // Waypoint reached
      local_wp_reached_ = true;
      ROS_WARN("Waypoint reached");
      if (curr_wp_info_.wp_index >= curr_wp_info_.total_wp - 1)
      {
        ROS_WARN("There are no more available waypoints");
        curr_wp_info_.wp_index = curr_wp_info_.total_wp - 1;
        curr_wp_info_.mission_completed = true;
        this->publish_wp_info();
      }
    }
    /*else
    {
      local_wp_reached_ = false;
    }*/

#ifdef DEBUG
    if (curr_wp_info_.wp_index >= curr_wp_info_.total_wp)
    {
      ROS_WARN("There are no more available waypoints");
    }
    ROS_INFO("Current info:");
    std::cout << "current heading diff [deg]: " << curr_heading_diff * 180.0 / M_PI << std::endl;
    std::cout << "previous heading diff [deg]: " << prev_heading_diff * 180.0 / M_PI << std::endl;
    std::cout << "current xy distance [m]: " << bg::distance(curr_p, goal_p) << std::endl;
    std::cout << "previous xy distance [m]: " << bg::distance(prev_p, goal_p) << std::endl;
    std::cout << "current z distance [m]: " << fabs(goal_local_alt_ - curr_local_alt_) << std::endl;
    std::cout << "previous z distance [m]: " << fabs(goal_local_alt_ - prev_local_alt_) << std::endl;

#endif
    this->publish_wp_info();

    ros::spinOnce();
    rate_.sleep();
  }
#ifdef DEBUG
  ROS_INFO_ONCE("Fcu disconnected or program terminated");
#endif
  return;
}

int main(int argc, char **argv)
{
  std::setprecision(10);
  ros::init(argc, argv, "wp_mgr_server");
  waypoint_manager_server wp_mgr_srvr = waypoint_manager_server();
  wp_mgr_srvr.run();
  return 0;
}