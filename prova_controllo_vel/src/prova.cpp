
#include "ros/ros.h"
#include "math.h"
#include "mavros_msgs/State.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

 mavros_msgs::State current_state;

 void stateCallback(const mavros_msgs::State::ConstPtr& msg){
     current_state = *msg;
 }




int main(int argc, char **argv)
{

  ros::init(argc, argv, "prova");


  ros::NodeHandle n;
  geometry_msgs::TwistStamped speed;


  ros::Publisher commandVel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);


  ros::Subscriber state_sub = n.subscribe("mavros/state", 10, stateCallback);

  ros::Rate loop_rate(10);
  while(ros::ok() && current_state.connected) {

       ros::spinOnce();
       loop_rate.sleep();

   }

////////////////////////////////////////////
/////////////////GUIDED/////////////////////
////////////////////////////////////////////
ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
mavros_msgs::SetMode srv_setMode;
srv_setMode.request.base_mode = 0;
srv_setMode.request.custom_mode = "GUIDED";
if(cl.call(srv_setMode)){
    // ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
}else{
    ROS_ERROR("Failed SetMode");
    return -1;
}

////////////////////////////////////////////
///////////////////ARM//////////////////////
////////////////////////////////////////////
ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
mavros_msgs::CommandBool srv;
srv.request.value = true;
if(arming_cl.call(srv)){
    // ROS_ERROR("ARM send ok %d", srv.response.success);
}else{
    ROS_ERROR("Failed arming or disarming");
}
sleep(1);
////////////////////////////////////////////
/////////////////TAKEOFF////////////////////
///////////////////////////////////////////
  ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 10;
  srv_takeoff.request.latitude = 1;
  srv_takeoff.request.longitude = 1;
  srv_takeoff.request.min_pitch = 1;
  srv_takeoff.request.yaw = 1;
  if(takeoff_cl.call(srv_takeoff)){
      ROS_ERROR("srv_takeoff send ok"); //, srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
sleep(15);
double count=0;
  while (ros::ok())
  {
    double w,r, t;

    w = 0.5;
    r = 20 ;
    t = w*count*0.05;

    speed.twist.linear.x = r*w*cos(t);
    speed.twist.linear.y = -r*w*sin(t);
    speed.twist.linear.z = 0;
    speed.twist.angular.x = 0;
    speed.twist.angular.y = 0;
    speed.twist.angular.z = 0;

    count++;


    commandVel_pub.publish(speed);

    ros::spinOnce();

    loop_rate.sleep();
  }

      ////////////////////////////////////////////
      ///////////////////LAND/////////////////////
      ////////////////////////////////////////////
      ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
      mavros_msgs::CommandTOL srv_land;
      srv_land.request.altitude = 0 ;
      srv_land.request.latitude = 1;
      srv_land.request.longitude = 1;
      srv_land.request.min_pitch = 1;
      srv_land.request.yaw = 1;
      if(land_cl.call(srv_land)){
          ROS_INFO("srv_land send ok");
      }else{
          ROS_ERROR("Failed Land");
      }

  return 0;
}
