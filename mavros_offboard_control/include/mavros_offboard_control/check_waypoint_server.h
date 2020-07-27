#ifndef CHECK_WAYPOINT_SERVER_H
#define CHECK_WAYPOINT_SERVER_H
#include  <iostream>
#include <stdio.h>
#include  <ros/ros.h>

#include  <mavros_offboard_control/Check_waypoint.h>

#include  <GeographicLib/Geodesic.hpp>


static const std::string kDefaultWpServerTopic = "offboard_controller/wp_server/check_wp";
static constexpr double kDefaultRate =20.0 ;

class check_waypoint_server
{
    ros::NodeHandle nh_;
    ros::ServiceServer  waypoint_controller_;
    std::string server_name_;
    ros::Time t_now_;
    ros::Time t_prev_;
    ros::Rate rate_;
public:
    check_waypoint_server();
    ~check_waypoint_server();
    bool  check_wp_cb(mavros_offboard_control::Check_waypoint::Request &req,
                      mavros_offboard_control::Check_waypoint::Response &res);
    void  Main_task();

};

#endif // CHECK_WAYPOINT_SERVER_H
