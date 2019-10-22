#include "ros/ros.h"
#include <math.h>
#include "trajectory_fitting/TargetPos.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "trajectory_generator");
   ros::NodeHandle nh;

   ros::Rate rate(10);

   ros::Publisher pub = nh.advertise<trajectory_fitting::TargetPos>("/trajectory_fitting/target_pos", 25);

   while (ros::ok())
   {
      trajectory_fitting::TargetPos tgt_pos;
      double t = ros::Time::now().toSec();
      tgt_pos.position.point.z = 5;
      tgt_pos.position.point.x = 30*(cos(0.21*t)/(1+pow((sin(0.21*t)), 2)));
      tgt_pos.position.point.y = 30*((cos(0.21*t)*sin(0.21*t))/(1+pow(sin(0.21*t),2)));
      tgt_pos.position.header.stamp = ros::Time::now();
      tgt_pos.distance = 5;


      pub.publish(tgt_pos);

      rate.sleep();
   }

   return 0;
}
