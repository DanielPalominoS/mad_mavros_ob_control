#include "mavros_offboard_control/sp_server.h"

setpoint_server::setpoint_server() : rate_(default_rate)
{
  // ros::NodeHandle pnh("~");
  // std::string sp_service_topic;
  // Get params if given
  /*double rate;
  pnh.param("rate", rate, default_rate);
  rate_ = ros::Rate(rate);*/
  target_pose_.coordinate_frame=target_pose_.FRAME_BODY_NED;
  target_pose_.position.x = 0;
  target_pose_.position.y = 0;
  target_pose_.position.z = 1;
  target_pose_.yaw = 0;

  target_pose_.yaw_rate = 0;
  target_pose_.velocity.x = 0;
  target_pose_.velocity.y = 0;
  target_pose_.velocity.z = 0;

  target_pose_.type_mask = 0;
  target_pose_.type_mask += target_pose_.IGNORE_VX;
  target_pose_.type_mask += target_pose_.IGNORE_VY;
  target_pose_.type_mask += target_pose_.IGNORE_VZ;

  target_pose_.type_mask += target_pose_.IGNORE_AFX;
  target_pose_.type_mask += target_pose_.IGNORE_AFY;
  target_pose_.type_mask += target_pose_.IGNORE_AFZ;
#ifdef DEBUG
  ROS_INFO_ONCE("Node initiated");
#endif

  // advertise services
  // nh_.advertiseService(wp_service_topic,boost::bind(&check_waypoint_server::check_wp_cb,this,_1),this);
  // sp_srvr_ = nh_.advertiseService(sp_service_topic, &setpoint_server::sp_cb, this);
  sp_srvr_ = nh_.advertiseService(default_sp_srvr_tp, &setpoint_server::sp_cb, this);

  local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(default_sp_lcl_pub_tp, 20);
  state_sub_ = nh_.subscribe<mavros_msgs::State>
          (default_state_subs_tp, 5,boost::bind(&setpoint_server::state_cb,this,_1));
}

setpoint_server::~setpoint_server()
{
}

bool setpoint_server::sp_cb(mavros_offboard_msgs::SetLocalSetpoint::Request &req,
                            mavros_offboard_msgs::SetLocalSetpoint::Response &res)
{
  bool success = false;
  target_pose_ = req.target_sp;
  success = true;
  res.success = success;
  return success;
}
void setpoint_server::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
}

void setpoint_server::init_node()
{
  // wait for FCU connection
  while (ros::ok() && !current_state_.connected)
  {
    ros::spinOnce();
    rate_.sleep();
  }
  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    target_pose_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(target_pose_);
    // local_pos_pub_.publish(TargetPose);
    ros::spinOnce();
    rate_.sleep();
  }
#ifdef DEBUG
  ROS_INFO_ONCE("Fcu connected");
#endif
  while (ros::ok() && current_state_.connected)
  {
    target_pose_.header.stamp = ros::Time::now();
    local_pos_pub_.publish(target_pose_);
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
  ros::init(argc, argv, "sp_server");
  setpoint_server sp_srvr = setpoint_server();
  sp_srvr.init_node();
  return 0;
}