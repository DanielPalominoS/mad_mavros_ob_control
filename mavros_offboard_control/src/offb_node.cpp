#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointClear.h>
#include <ros/ros.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher local_target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 20);
  //ros::Publisher global_pos_pub =
  //    nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 20);
  //ros::Publisher global_target_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 20);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(200.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 10;

  mavros_msgs::PositionTarget TargetPose;
  TargetPose.coordinate_frame = TargetPose.FRAME_LOCAL_NED;
  TargetPose.position.x = 0;
  TargetPose.position.y = 0;
  TargetPose.position.z = 10;
  TargetPose.yaw = 0;
  TargetPose.yaw_rate = 2;
  TargetPose.velocity.x = 0;
  TargetPose.velocity.y = 0;
  TargetPose.velocity.z = 20;
  TargetPose.type_mask = 0;
  TargetPose.type_mask += TargetPose.IGNORE_AFX;
  TargetPose.type_mask += TargetPose.IGNORE_AFY;
  TargetPose.type_mask += TargetPose.IGNORE_AFZ;
  TargetPose.type_mask += TargetPose.IGNORE_VX;
  TargetPose.type_mask += TargetPose.IGNORE_VY;
  TargetPose.type_mask += TargetPose.IGNORE_VZ;


  mavros_msgs::GlobalPositionTarget GlobalTargetPose;
  GlobalTargetPose.coordinate_frame = 0;
  // GlobalTargetPose.altitude=535.276919+120;
  GlobalTargetPose.altitude = 10;
  GlobalTargetPose.latitude = 47.3977425;
  GlobalTargetPose.longitude = 8.5455935;
  // GlobalTargetPose.yaw=0;//1.570756;
  // GlobalTargetPose.yaw=1.570756;
  GlobalTargetPose.yaw = -1.570756;
  // GlobalTargetPose.yaw=-2.35619449;
  GlobalTargetPose.yaw_rate = 2;
  GlobalTargetPose.velocity.x = 0;
  GlobalTargetPose.velocity.y = 0;
  GlobalTargetPose.velocity.z = 20; 
  // GlobalTargetPose.acceleration_or_force.x=10;
  // GlobalTargetPose.acceleration_or_force.y=10;
  // GlobalTargetPose.acceleration_or_force.z=10;
  // GlobalTargetPose.acceleration_or_force.x=10;
  // GlobalTargetPose.coordinate_frame=mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
  // GlobalTargetPose.type_mask=960;//mavros_msgs::GlobalPositionTarget::IGNORE_AFX+mavros_msgs::GlobalPositionTarget::IGNORE_AFY+mavros_msgs::GlobalPositionTarget::IGNORE_AFZ+mavros_msgs::GlobalPositionTarget::FORCE;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    // GlobalTargetPose.header.stamp = ros::Time::now();
    // local_pos_pub.publish(pose);

    TargetPose.header.stamp = ros::Time::now();
    local_target_pos_pub.publish(TargetPose);
    // global_pos_pub.publish(GlobalTargetPose);
    // global_target_pub.publish(GlobalTargetPose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        // http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack Lista de modos disponibles
        ROS_INFO("OFFBOARD enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    //GlobalTargetPose.header.stamp = ros::Time::now();
    TargetPose.header.stamp = ros::Time::now();
    // global_pos_pub.publish(GlobalTargetPose);
    // global_target_pub.publish(GlobalTargetPose);
    local_target_pos_pub.publish(TargetPose);
    // local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
