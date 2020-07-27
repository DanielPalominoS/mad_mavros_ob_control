#include <ros/rate.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

// Mavros offboard msgs (own generated)
#include "mavros_offboard_msgs/CurrentWaypointInfo.h"
#include "mavros_offboard_msgs/GetWaypointList.h"
#include "mavros_offboard_msgs/LocalWaypoint.h"
#include "mavros_offboard_msgs/LocalWaypointList.h"
#include "mavros_offboard_msgs/ModifyWaypointList.h"
#include "mavros_offboard_msgs/MoveToNextWaypoint.h"
#include "mavros_offboard_msgs/SetLocalSetpoint.h"
#include "mavros_offboard_msgs/SetWaypointThreshold.h"

static const std::string dflt_current_wp_pub_tp = "wp_manager/current_waypoint";

static const std::string dflt_sp_srvr_tp = "sp_server/set_setpoint";

static const std::string dflt_modify_wp_list_srv_tp = "wp_manager/modify_wp_list";
static const std::string dflt_get_wp_list_srv_tp = "wp_manager/get_waypoint_list";
static const std::string dflt_move_to_next_wp_srv_tp = "wp_manager/move_to_next_waypoint";

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

mavros_offboard_msgs::CurrentWaypointInfo wp_info;
void curr_wp_info_cb(const mavros_offboard_msgs::CurrentWaypointInfo::ConstPtr& msg)
{
  wp_info = *msg;
}

void update_sp_object_pose(mavros_offboard_msgs::SetLocalSetpoint* sp_obj, double x, double y, double z, double yaw)
{
  sp_obj->request.target_sp.position.x = x;
  sp_obj->request.target_sp.position.y = y;
  sp_obj->request.target_sp.position.z = z;
  sp_obj->request.target_sp.yaw = yaw;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_ob_node");
  ros::NodeHandle nh_;
  wp_info.mission_completed = false;

  // Subscribers
  ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber curr_wp_info_sub =
      nh_.subscribe<mavros_offboard_msgs::CurrentWaypointInfo>(dflt_current_wp_pub_tp, 10, curr_wp_info_cb);

  // Service clients
  ros::ServiceClient set_sp_client_ = nh_.serviceClient<mavros_offboard_msgs::SetLocalSetpoint>(dflt_sp_srvr_tp);

  ros::ServiceClient modify_wp_list_client =
      nh_.serviceClient<mavros_offboard_msgs::ModifyWaypointList>(dflt_modify_wp_list_srv_tp);

  ros::ServiceClient get_wp_list_client_ =
      nh_.serviceClient<mavros_offboard_msgs::GetWaypointList>(dflt_get_wp_list_srv_tp);

  ros::ServiceClient move_to_next_wp_client_ =
      nh_.serviceClient<mavros_offboard_msgs::MoveToNextWaypoint>(dflt_move_to_next_wp_srv_tp);

  ros::Rate rate(40.0);

  mavros_offboard_msgs::SetLocalSetpoint set_sp_obj;
  mavros_offboard_msgs::ModifyWaypointList modify_wp_obj;
  mavros_offboard_msgs::GetWaypointList get_wp_list_obj;
  mavros_offboard_msgs::MoveToNextWaypoint move_to_next_obj;
  mavros_offboard_msgs::LocalWaypointList lcl_wp_list;

  set_sp_obj.request.target_sp.type_mask = 504;
  set_sp_obj.request.target_sp.coordinate_frame = 7;

  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

#ifdef DEBUG
  ROS_INFO_ONCE("FCU connected");
#endif

  // Get default wp_list
  while (!get_wp_list_client_.call(get_wp_list_obj) && ros::ok())
  {
#ifdef DEBUG
    ROS_INFO_ONCE("Waitting wp_list");

#endif
  }
#ifdef DEBUG
  ROS_INFO_ONCE("list adquired");

#endif
  lcl_wp_list = get_wp_list_obj.response.local_wp_list;
  int num_wp = lcl_wp_list.local_waypoints.size();
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    num_wp = lcl_wp_list.local_waypoints.size();
    for (int i = 0; i < 1; i++)
    {
      lcl_wp_list.current_seq = 0;
      modify_wp_obj.request.new_wp_list = lcl_wp_list;
      wp_info.mission_completed = false;
      wp_info.wp_index = 0;
      modify_wp_list_client.call(modify_wp_obj);

      /*for (int i = 0; i < num_wp; i++)
      {
        ros::spinOnce();
        rate.sleep();
        update_sp_object_pose(&set_sp_obj, wp_info.current_wp.local_waypoint.x, wp_info.current_wp.local_waypoint.y,
                              wp_info.current_wp.local_waypoint.z, wp_info.current_wp.heading);
        set_sp_client_.call(set_sp_obj);
  #ifdef DEBUG
        ROS_INFO("approaching");
        std::cout << "waypoint" << wp_info.wp_index << "\n" << wp_info.current_wp << std::endl;
  #endif
        while (!wp_info.reached && ros::ok())
        {
          ros::spinOnce();
          rate.sleep();
        }
        ros::Duration(1.0).sleep();
  #ifdef DEBUG
        ROS_INFO("waypoint reached");
  #endif
        move_to_next_wp_client_.call(move_to_next_obj);
        if (!move_to_next_obj.response.success)
          break;

        ros::spinOnce();
        rate.sleep();
      }*/
      while (!wp_info.mission_completed && ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
        update_sp_object_pose(&set_sp_obj, wp_info.current_wp.local_waypoint.x, wp_info.current_wp.local_waypoint.y,
                              wp_info.current_wp.local_waypoint.z, wp_info.current_wp.heading);
        set_sp_client_.call(set_sp_obj);
#ifdef DEBUG
        ROS_INFO("approaching");
        std::cout << "waypoint " << wp_info.wp_index << "\n" << wp_info.current_wp << std::endl;
#endif
        while (!wp_info.reached && ros::ok())
        {
          ros::spinOnce();
          rate.sleep();
        }
        ros::Duration(1.0).sleep();
#ifdef DEBUG
        ROS_INFO("waypoint reached");
#endif
        move_to_next_wp_client_.call(move_to_next_obj);
        if (!move_to_next_obj.response.success)
          break;

        ros::spinOnce();
        rate.sleep();
      }
    }
    set_sp_obj.request.target_sp.position.x = 0;
    set_sp_obj.request.target_sp.position.y = 0;
    set_sp_obj.request.target_sp.position.z = 0;
    set_sp_obj.request.target_sp.yaw=M_PI_2;
    set_sp_client_.call(set_sp_obj);
    
#ifdef DEBUG
    ROS_INFO("Routine ended");
#endif
    break;
  }
}