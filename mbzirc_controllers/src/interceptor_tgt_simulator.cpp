#include "ros/ros.h"
#include <math.h>
#include<cstdlib>
#include <iostream>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Point.h"

#define NO_SMACH 1

geometry_msgs::Point     tgtPos;
bool clear=false;   
std_msgs::String task_id;
float globalTime=0.1;

void setTgtPos(float t){
  //TRAJ 1: GETTIN FAR AWAY IN A LINE
  tgtPos.x=0;
  tgtPos.y=2;
  tgtPos.z=10+0.3*t;
}

void clearanceCallaback( const std_msgs::Bool::ConstPtr& msg ) {
  clear=msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interceptor_tgt_simulator");
  ros::NodeHandle n;

  // CREATE ROS PUBLISH OBJECTS
  ros::Publisher tgtPos_pub = n.advertise<geometry_msgs::Point>("target_pos", 1);
  ros::Publisher taskId_pub = n.advertise<std_msgs::String>("/smach/task_id", 3);

  // SUBSCRIBE TO TOPICS
  ros::Subscriber clearance_sub = n.subscribe("sim_clearance",1, clearanceCallaback);

  ros::Rate loop_rate(10);

  //tgtPos=geometry_msgs::Point(0,2,10); CONSTRUCTOR?? //assuming: x to the right, y to the up and z forward in depth
  task_id.data = "IDLE";
  
  double          secs_start;
  while(ros::ok() && !clear){
    if(NO_SMACH){
      secs_start = ros::Time::now().toSec();
      while(ros::Time::now().toSec()-secs_start<=5){;
        std::cout<< "waiting\n" ;//debug;
      }
      
      clear=true;  
    }
    else
    {
      ros::spinOnce();
    }
  }
  //we have now been cleared to publish the target position!
  //std::stringstream ss;
  //ss<<"CHASE";
  task_id.data="CHASE";
  taskId_pub.publish(task_id);

  while ( ros::ok() )
  {
    setTgtPos(globalTime);
    tgtPos_pub.publish(tgtPos);

    globalTime+=0.1;
    ros::spinOnce(); //call all the callbacks once
    loop_rate.sleep();

  }
  return 0;
}
