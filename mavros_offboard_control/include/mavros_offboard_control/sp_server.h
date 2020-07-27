#ifndef SETPOINT_SERVER_H
#define SETPOINT_SERVER_H
#include <iostream>
#include <stdio.h> 
#include <ros/ros.h>
#include <ros/rate.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <mavros_offboard_msgs/SetLocalSetpoint.h>


static const std::string default_sp_srvr_tp = "sp_server/set_setpoint";
static constexpr double default_rate = 20.0;
static const std::string default_sp_lcl_pub_tp = "mavros/setpoint_raw/local";
static const std::string default_state_subs_tp = "mavros/state";


class setpoint_server
{
    ros::NodeHandle nh_;
    ros::ServiceServer sp_srvr_;
    std::string srvr_name_;
    ros::Time t_now_;
    ros::Time t_prev_;
    ros::Rate rate_;

    mavros_msgs::State current_state_;

    ros::Subscriber state_sub_;

    mavros_msgs::PositionTarget target_pose_;

    ros::Publisher local_pos_pub_;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);

public:
    setpoint_server();
    ~setpoint_server();
    bool sp_cb(mavros_offboard_msgs::SetLocalSetpoint::Request &req,
               mavros_offboard_msgs::SetLocalSetpoint::Response &res);
    void init_node();
};

#endif // SETPOINT_SERVER_