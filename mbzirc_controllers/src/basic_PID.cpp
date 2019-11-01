#include "ros/ros.h"
#include <math.h>
#include<cstdlib>
#include <iostream>
 #include </usr/local/include/eigen3/Eigen/Core>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Point.h"
#include "mbzirc_controller/triplePIDparam.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>


Eigen::VectorXd     targetPos(3);
Eigen::VectorXd     windupMax(3);
Eigen::VectorXd     errorI(3);
Eigen::VectorXd     dronePose(7);
Eigen::VectorXd     droneTwist(6);
Eigen::MatrixXd     Kp(3,3) , Kd(3,3), Ki(3,3);
std::vector<double> K;
int                 derInit;
bool                searchInit;
std::string task_id;



void targetPosCallback( const geometry_msgs::Point::ConstPtr& target_pos ) {
  // Update the target state of the AUV
  targetPos(0) = target_pos->x;
  targetPos(1) = target_pos->y;
  targetPos(2) = target_pos->z;
}


void PIDparamSetCallback( const mbzirc_controller::triplePIDparam::ConstPtr& K_PID )  {

  Kp(2,2) = K_PID->surge.Kp;
  Kd(2,2) = K_PID->surge.Kd;
  Ki(2,2) = K_PID->surge.Ki;
  Kp(1,1) = K_PID->height.Kp;
  Kd(1,1) = K_PID->height.Kd;
  Ki(1,1) = K_PID->height.Ki;
  Kp(0,0) = K_PID->yaw.Kp;
  Kd(0,0) = K_PID->yaw.Kd;
  Ki(0,0) = K_PID->yaw.Ki;
  errorI << 0,0,0;   // initlize integral error to 0 every time a new param assignation
  derInit = 0;       // to avoid spikes of the first errorD

}



void TaskIDCallback( const std_msgs::String::ConstPtr& task )  {

    task_id = task->data;
    searchInit = 1;

}

 mavros_msgs::State current_state;

 void stateCallback(const mavros_msgs::State::ConstPtr& msg){
     current_state = *msg;
 }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry){
      dronePose(0) = odometry->pose.pose.position.x;
      dronePose(1) = odometry->pose.pose.position.y;
      dronePose(2) = odometry->pose.pose.position.z;
      dronePose(3) = odometry->pose.pose.orientation.x;
      dronePose(4) = odometry->pose.pose.orientation.y;
      dronePose(5) = odometry->pose.pose.orientation.z;
      dronePose(6) = odometry->pose.pose.orientation.w;
      droneTwist(0) = odometry->twist.twist.linear.x;
      droneTwist(1) = odometry->twist.twist.linear.y;
      droneTwist(2) = odometry->twist.twist.linear.z;
      droneTwist(3) = odometry->twist.twist.angular.x;
      droneTwist(4) = odometry->twist.twist.angular.y;
      droneTwist(5) = odometry->twist.twist.angular.z;
  }

// Anti windup system
Eigen::VectorXd antiWindup(Eigen::VectorXd u, Eigen::VectorXd windupMax)
{
  for( int i=0; i<3; i++){
    if (u(i) >= windupMax(i))
    {
        u(i) = windupMax(i);
    }
    else if (u(i) <= -windupMax(i))
    {
        u(i) = -windupMax(i);
    }
  }
  return u;
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "basic_PID");

  ros::NodeHandle n;

  // CREATE ROS PUBLISH OBJECTS
  ros::Publisher commandVel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
  ros::Publisher commandPos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  ros::Publisher PID_pub = n.advertise<mbzirc_controller::triplePIDparam>("PID_param", 1);

  // SUBSCRIBE TO TOPICS
  ros::Subscriber subtaskSub  = n.subscribe("PIDparamSet", 1, PIDparamSetCallback);
  ros::Subscriber targetPosSub = n.subscribe("target_pos",1, targetPosCallback);
  ros::Subscriber state_sub = n.subscribe("mavros/state", 1, stateCallback);
  ros::Subscriber taskID_sub = n.subscribe("/smach/task_id", 1, TaskIDCallback);
  ros::Subscriber odom_sub = n.subscribe("/mavros/local_position/odom", 1, OdomCallback);

  ros::Rate loop_rate(10);

  geometry_msgs::TwistStamped           velRef;
  geometry_msgs::PoseStamped            posRef;
  mbzirc_controller::triplePIDparam     PIDparam;
  std::vector<double>                   windupMaxv;

  Eigen::VectorXd error(3);
  Eigen::VectorXd errorD(3);
  Eigen::VectorXd errorOld(3);
  Eigen::VectorXd reference(3);
  Eigen::VectorXd vel_ref(3);
  double          secs_start;
  double          secs_fin;
  double          Dt = 0;

  targetPos << 0,0,0;
  errorOld << 0,0,0;
  errorI   << 0,0,0;
  task_id = "IDLE";

  ////          DA AGGIUSTARE!!!         ////

  derInit = 0;
  searchInit = 1;
  int wpCounter = 0;


  Kp << Eigen::MatrixXd::Zero(3,3);
  Kd << Eigen::MatrixXd::Zero(3,3);
  Ki << Eigen::MatrixXd::Zero(3,3);

  windupMax << 0.1, 0.1, 0.1;



  while ( ros::ok() )
  {

    if( !strcmp(task_id.c_str(),"SEARCH" ) ) {

        if(searchInit)
          secs_start = ros::Time::now().toSec();
        secs_fin = ros::Time::now().toSec();
        Dt = secs_fin - secs_start;
        //                  DA AGGIUSTARE !!!!!!!!
        // La gestione coi tempi è un po' una porcata, in futuro meglio mettere
        // un altro tipo di parametro magari posizionale o simile
        if( Dt<10 )  {
        velRef.twist.linear.x = 0;
        velRef.twist.linear.y = 0;
        velRef.twist.linear.z = 0;
        velRef.twist.angular.x = 0;
        velRef.twist.angular.y = 0;
        velRef.twist.angular.z = 1;
        commandVel_pub.publish(velRef);
        }
        else if ( Dt<13 ) {
            //              DA AGGIUSTARE !!!!!
            //  Meglio mettere i waypoint come messaggio o parametro in modo che
            //  siano più accessibili

            velRef.twist.linear.z = 0;
            velRef.twist.angular.x = 0;
            velRef.twist.angular.y = 0;
            velRef.twist.angular.z = 0;

            if( wpCounter % 4 == 0) {
            velRef.twist.linear.x = 2;
            velRef.twist.linear.y = 0;
            }
            else if( wpCounter % 4 == 1) {
            velRef.twist.linear.x = 0;
            velRef.twist.linear.y = 2;
            }

            else if( wpCounter % 4 == 2) {
            velRef.twist.linear.x = -2;
            velRef.twist.linear.y = 0;
            }

            else if( wpCounter % 4 == 3) {
            velRef.twist.linear.x = 0;
            velRef.twist.linear.y = -2;
            }

            commandVel_pub.publish(velRef);
        }
        // else if ( Dt<15 ) {
        //     //              DA AGGIUSTARE !!!!!
        //     //  Meglio mettere i waypoint come messaggio o parametro in modo che
        //     //  siano più accessibili
        //     posRef.pose.position.z = 3;
        //     if( wpCounter % 4 == 0) {
        //         posRef.pose.position.x = 4;
        //         posRef.pose.position.y = 4;
        //     }
        //     else if( wpCounter % 4 == 1) {
        //         posRef.pose.position.x = 4;
        //         posRef.pose.position.y = -4;
        //     }
        //
        //     else if( wpCounter % 4 == 2) {
        //         posRef.pose.position.x = -4;
        //         posRef.pose.position.y = -4;
        //     }
        //
        //     else if( wpCounter % 4 == 3) {
        //         posRef.pose.position.x = -4;
        //         posRef.pose.position.y = 4;
        //     }
        //
        //     commandPos_pub.publish(posRef);
        // }
        else  {
            secs_start = ros::Time::now().toSec();
            wpCounter++;

        }
    searchInit=0;
    } //end of search state if

    if( !strcmp(task_id.c_str(),"CHASE" ) ) {

        double dt;
        if(derInit) dt = secs_fin - secs_start;
        else dt = 0.1;
        secs_start = ros::Time::now().toSec();

        error = - targetPos;

        // Compute errors, errors derivative and integral
        errorD   = (error - errorOld)/(dt)*derInit;
        errorI   = errorI + error*dt;
        errorI   = antiWindup(errorI, windupMax);
        errorOld = error;
        derInit  = 1;

        //MIMO PID
        vel_ref = Kp*error + Kd*errorD + Ki*errorI;

        //Update ros message


        velRef.twist.linear.x = vel_ref(2);
        velRef.twist.linear.y = 0;
        velRef.twist.linear.z = vel_ref(1);
        velRef.twist.angular.x = 0;
        velRef.twist.angular.y = 0;
        velRef.twist.angular.z = vel_ref(0);

        // std::cout << "Here is the vector v:\n" << vel_ref << std::endl;
        // std::cout << "Here is the vector error\n" << error << std::endl;

        PIDparam.surge.Kp = Kp(0,0);
        PIDparam.surge.Kd = Kd(0,0);
        PIDparam.surge.Ki = Ki(0,0);
        PIDparam.height.Kp = Kp(1,1);
        PIDparam.height.Kd = Kd(1,1);
        PIDparam.height.Ki = Ki(1,1);
        PIDparam.yaw.Kp = Kp(2,2);
        PIDparam.yaw.Kd = Kd(2,2);
        PIDparam.yaw.Ki = Ki(2,2);

        commandVel_pub.publish(velRef);

    }

    //Publish the updated data
    PID_pub.publish(PIDparam);
    ros::spinOnce();
    loop_rate.sleep();
    secs_fin = ros::Time::now().toSec();
  }//end of ros main loop


  return 0;
}
