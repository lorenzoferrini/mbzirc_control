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
std::string task_id="YETINNIT";
float globalTime=0.1;

void setTgtPos(float t){
  //TRAJ 1: GETTIN FAR AWAY IN A LINE
  tgtPos.x=0.3*t;
  tgtPos.y=2;
  tgtPos.z=3;
}

void clearanceCallaback( const std_msgs::Bool::ConstPtr& msg ) {
  clear=msg->data;
}


void TaskIDCallback( const std_msgs::String::ConstPtr& task )  {

    task_id = task->data;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "interceptor_tgt_simulator");
  ros::NodeHandle n;

  // CREATE ROS PUBLISH OBJECTS
  ros::Publisher tgtPos_pub = n.advertise<geometry_msgs::Point>("target_pos", 1);
  ros::Publisher tgtfound_pub = n.advertise<std_msgs::Bool>("/target_found", 3);

  // SUBSCRIBE TO TOPICS
  ros::Subscriber clearance_sub = n.subscribe("sim_clearance",1, clearanceCallaback);
  ros::Subscriber taskID_sub = n.subscribe("/smach/task_id", 1, TaskIDCallback);

  ros::Rate loop_rate(10);

  //tgtPos=geometry_msgs::Point(0,2,10); CONSTRUCTOR?? //assuming: x to the right, y to the up and z forward in depth

  double          secs_start;
  while(ros::ok() && strcmp(task_id.c_str(),"SEARCH" )){
        ros::spinOnce();
        std::cout << task_id << std::endl;
  }
  //we have now been cleared to publish the target position!
  //std::stringstream ss;
  //ss<<"CHASE";

    std_msgs::Bool tgt_found;
    tgt_found.data = true;
  while ( ros::ok() )
  {
    tgtfound_pub.publish(tgt_found);
    setTgtPos(globalTime);
    tgtPos_pub.publish(tgtPos);

    globalTime+=0.1;
    ros::spinOnce(); //call all the callbacks once
    loop_rate.sleep();

  }
  return 0;
}
