
#include "ros/ros.h"
#include <math.h>
#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Point.h"
#include "mbzirc_control/triplePIDparam.h"
#include <geometry_msgs/TwistStamped.h>


Eigen::VectorXd     targetPos(3);
Eigen::VectorXd     windupMax(6);
Eigen::VectorXd     errorI(6);
Eigen::MatrixXd     Kp(3,3) , Kd(3,3), Ki(3,3);
std::vector<double> K;
int                 derInit;



void targetPosCallback( const geometry_msgs::Point::ConstPtr& target_pos ) {
  // Update the target state of the AUV
  targetPos(0) = target_pos->x;
  targetPos(1) = target_pos->y;
  targetPos(2) = target_pos->z;
}


void PIDparamSetCallback( const mbzirc_control::triplePIDparam::ConstPtr& K_PID )  {

  Kp(1,1) = K_PID->surge.Kp;
  Kd(1,1) = K_PID->surge.Kd;
  Ki(1,1) = K_PID->surge.Ki;
  Kp(2,2) = K_PID->height.Kp;
  Kd(2,2) = K_PID->height.Kd;
  Ki(2,2) = K_PID->height.Ki;
  Kp(3,3) = K_PID->yaw.Kp;
  Kd(3,3) = K_PID->yaw.Kd;
  Ki(3,3) = K_PID->yaw.Ki;
  errorI << 0,0,0;   // initlize integral error to 0 every time a new param assignation
  derInit = 0;       // to avoid spikes of the first errorD

}

 mavros_msgs::State current_state;

 void stateCallback(const mavros_msgs::State::ConstPtr& msg){
     current_state = *msg;
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
  ros::Publisher PID_pub = n.advertise<mbzirc_control::triplePIDparam>("PID_param", 1);

  // SUBSCRIBE TO TOPICS
  ros::Subscriber subtaskSub  = n.subscribe("PIDparamSet", 1, PIDparamSetCallback);
  ros::Subscriber targetPosSub = n.subscribe("targetPos",1, targetPosCallback);
  ros::Subscriber state_sub = n.subscribe("mavros/state", 1, stateCallback);

  ros::Rate loop_rate(10);

  geometry_msgs::TwistStamped      velRef;
  mbzirc_control::triplePIDparam   PIDparam;
  std::vector<double>              windupMaxv;

  Eigen::VectorXd error(3);
  Eigen::VectorXd errorD(3);
  Eigen::VectorXd errorOld(3);
  Eigen::VectorXd reference(3);
  Eigen::VectorXd vel_ref(3);
  double          secs_start;
  double          secs_fin;

  targetPos << 0,400,400;
  errorOld << 0,0,0;
  errorI   << 0,0,0;

  ////          DA AGGIUSTARE!!!         ////
  reference << 0,400,400; //i primi due dovrebbero essere le coordinate in pixel del centro frame
  derInit   = 0;

  Kp << Eigen::MatrixXd::Zero(3,3);
  Kd << Eigen::MatrixXd::Zero(3,3);
  Ki << Eigen::MatrixXd::Zero(3,3);

  windupMax << 0.1, 0.1, 0.1;



  while ( ros::ok() )
  {
    velRef.twist.linear.x = 0;
    velRef.twist.linear.y = 0;
    velRef.twist.linear.z = 0;
    velRef.twist.angular.x = 0;
    velRef.twist.angular.y = 0;
    velRef.twist.angular.z = 0;

    double dt;
    if(derInit) dt = secs_fin - secs_start;
    else dt = 0.1;
    secs_start = ros::Time::now().toSec();

    error = reference - targetPos;

    // Compute errors, errors derivative and integral
    errorD   = (error - errorOld)/(dt)*derInit;
    errorI   = errorI + error*dt;
    errorI   = antiWindup(errorI, windupMax);
    errorOld = error;
    derInit  = 1;

    //MIMO PID
    vel_ref = Kp*error + Kd*errorD + Ki*errorI;

    //Update ros message


    velRef.twist.linear.x = vel_ref(0);
    velRef.twist.linear.y = 0;
    velRef.twist.linear.z = vel_ref(1);
    velRef.twist.angular.x = 0;
    velRef.twist.angular.y = 0;
    velRef.twist.angular.z = vel_ref(2);

    PIDparam.surge.Kp = Kp(0,0);
    PIDparam.surge.Kd = Kd(0,0);
    PIDparam.surge.Ki = Ki(0,0);
    PIDparam.height.Kp = Kp(1,1);
    PIDparam.height.Kd = Kd(1,1);
    PIDparam.height.Ki = Ki(1,1);
    PIDparam.yaw.Kp = Kp(2,2);
    PIDparam.yaw.Kd = Kd(2,2);
    PIDparam.yaw.Ki = Ki(2,2);


    //Publish the updated data
    commandVel_pub.publish(velRef);
    PID_pub.publish(PIDparam);
    ros::spinOnce();
    loop_rate.sleep();
    secs_fin = ros::Time::now().toSec();
  }


  return 0;
}
