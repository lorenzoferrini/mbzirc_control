#include "ros/ros.h"
#include <math.h>
#include<cstdlib>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Point.h"
#include "mbzirc_controller/triplePIDparam.h"
#include "mbzirc_controller/directionalPIDparam.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>


class DirectionalPID    {
public:
    DirectionalPID();
    void TgtPosCallback( const geometry_msgs::Point::ConstPtr& target_pos);
    void PIDparamSetCallback( const mbzirc_controller::directionalPIDparam::ConstPtr& K_PID );
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry);
    void TaskIDCallback( const std_msgs::String::ConstPtr& task );
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    Eigen::VectorXd antiWindup(Eigen::VectorXd u, Eigen::VectorXd windupMax);
    void SpeedControl();
    void Timeupdate();

private:

    ros::NodeHandle n = ros::NodeHandle("~");
    ros::Publisher commandVel_pub;
    ros::Publisher commandPos_pub;
    ros::Publisher PID_pub;
    ros::Subscriber subtaskSub;
    ros::Subscriber targetPosSub;
    ros::Subscriber state_sub;
    ros::Subscriber taskID_sub;
    ros::Subscriber odom_sub;

    Eigen::VectorXd     targetPos;
    Eigen::VectorXd     targetPos_NORMALIZED;
    Eigen::VectorXd     windupMax;
    Eigen::VectorXd     errorI;
    Eigen::VectorXd     dronePos;
    Eigen::Quaterniond  droneQuat;
    Eigen::VectorXd     droneTwist;
    Eigen::MatrixXd     Kp;
    Eigen::MatrixXd     Ki;
    Eigen::MatrixXd     Kd;
    Eigen::MatrixXd     R;
    std::vector<double> K;
    int                 derInit;
    bool                searchInit;
    int                 wpCounter;
    std::string task_id;
    mavros_msgs::State current_state;

    geometry_msgs::TwistStamped             velRef;
    geometry_msgs::PoseStamped              posRef;
    mbzirc_controller::directionalPIDparam  PIDparam;
    std::vector<double>                     windupMaxv;


    Eigen::VectorXd error;
    Eigen::VectorXd errorD;
    Eigen::VectorXd errorOld;
    Eigen::VectorXd reference;
    Eigen::VectorXd vel_ref;
    Eigen::VectorXd xAxis;
    Eigen::VectorXd yAxis;
    double          secs_start;
    double          secs_fin;
    double          Dt;


};

DirectionalPID::DirectionalPID() {


  // CREATE ROS PUBLISH OBJECTS
  commandVel_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
  commandPos_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
  PID_pub = n.advertise<mbzirc_controller::directionalPIDparam>("PID_param", 1);

  // SUBSCRIBE TO TOPICS
  subtaskSub  = n.subscribe("PIDparamSet", 1, &DirectionalPID::PIDparamSetCallback, this);
  targetPosSub = n.subscribe("/target_pos",1, &DirectionalPID::TgtPosCallback, this);
  state_sub = n.subscribe("/mavros/state", 1, &DirectionalPID::stateCallback, this);
  taskID_sub = n.subscribe("/smach/task_id", 1, &DirectionalPID::TaskIDCallback, this);
  odom_sub = n.subscribe("/mavros/local_position/odom", 1, &DirectionalPID::OdomCallback, this);

  targetPos = Eigen::VectorXd(3);
  windupMax = Eigen::VectorXd(2);
  errorI    = Eigen::VectorXd(2);
  dronePos  = Eigen::VectorXd(3);
  droneTwist  = Eigen::VectorXd(6);
  Kp  = Eigen::MatrixXd::Zero(2,2);
  Kd  = Eigen::MatrixXd::Zero(2,2);
  Ki  = Eigen::MatrixXd::Zero(2,2);
  R  = Eigen::MatrixXd(3,3);
  error  = Eigen::VectorXd(2);
  errorD  = Eigen::VectorXd(2);
  errorOld  = Eigen::VectorXd(2);
  reference  = Eigen::VectorXd(2);
  vel_ref  = Eigen::VectorXd(2);
  xAxis = Eigen::VectorXd(3);
  yAxis = Eigen::VectorXd(3);
  Dt = 0;

  windupMax << 0.1, 0.1;

  ////          DA AGGIUSTARE!!!         ////

  derInit = 0;
  searchInit = 1;
  wpCounter = 0;

  targetPos << 0,0,0;
  errorOld << 0,0;
  errorI   << 0,0;
  task_id = "IDLE";

}


void DirectionalPID::TgtPosCallback( const geometry_msgs::Point::ConstPtr& target_pos)  {

  targetPos(0) = target_pos->x;
  targetPos(1) = target_pos->y;
  targetPos(2) = target_pos->z;

  //DA RIMUOVERE CON LA CV

  targetPos = targetPos - dronePos;

}


void DirectionalPID::PIDparamSetCallback( const mbzirc_controller::directionalPIDparam::ConstPtr& K_PID )  {

    Kp(1,1) = K_PID->surge.Kp;
    Kd(1,1) = K_PID->surge.Kd;
    Ki(1,1) = K_PID->surge.Ki;
    Kp(0,0) = K_PID->yaw.Kp;
    Kd(0,0) = K_PID->yaw.Kd;
    Ki(0,0) = K_PID->yaw.Ki;
    errorI << 0,0;   // initlize integral error to 0 every time a new param assignation
    derInit = 0;       // to avoid spikes of the first errorD

}

void DirectionalPID::TaskIDCallback( const std_msgs::String::ConstPtr& task )   {
    task_id = task->data;
    searchInit = 1;
}


void DirectionalPID::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void DirectionalPID::OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry){
    dronePos(0) = odometry->pose.pose.position.x;
    dronePos(1) = odometry->pose.pose.position.y;
    dronePos(2) = odometry->pose.pose.position.z;
    droneQuat.x() = odometry->pose.pose.orientation.x;
    droneQuat.y() = odometry->pose.pose.orientation.y;
    droneQuat.z() = odometry->pose.pose.orientation.z;
    droneQuat.w() = odometry->pose.pose.orientation.w;
    droneTwist(0) = odometry->twist.twist.linear.x;
    droneTwist(1) = odometry->twist.twist.linear.y;
    droneTwist(2) = odometry->twist.twist.linear.z;
    droneTwist(3) = odometry->twist.twist.angular.x;
    droneTwist(4) = odometry->twist.twist.angular.y;
    droneTwist(5) = odometry->twist.twist.angular.z;
}

Eigen::VectorXd DirectionalPID::antiWindup(Eigen::VectorXd u, Eigen::VectorXd windupMax)
{
  for( int i=0; i<2; i++){
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


void DirectionalPID::Timeupdate()   {
secs_fin = ros::Time::now().toSec();
}

void DirectionalPID::SpeedControl() {

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


      targetPos_NORMALIZED = targetPos.normalized(); // versore drone-target
      R = droneQuat.normalized().toRotationMatrix();
      xAxis = R * Eigen::Vector3d{1, 0, 0};
      yAxis = R * Eigen::Vector3d{0, 1, 0};

      error(0) = atan2(targetPos.dot(yAxis), targetPos.dot(xAxis));
      error(1) = targetPos.norm();

      // Compute errors, errors derivative and integral
      errorD   = (error - errorOld)/(dt)*derInit;
      errorI   = errorI + error*dt;
      errorI   = antiWindup(errorI, windupMax);
      errorOld = error;
      derInit  = 1;

      //MIMO PID
      vel_ref = Kp*error + Kd*errorD + Ki*errorI;

      //Update ros message

      velRef.twist.linear.x = vel_ref(1)*targetPos_NORMALIZED(0);
      velRef.twist.linear.y = vel_ref(1)*targetPos_NORMALIZED(1);
      velRef.twist.linear.z = vel_ref(1)*targetPos_NORMALIZED(2);
      velRef.twist.angular.x = 0;
      velRef.twist.angular.y = 0;
      velRef.twist.angular.z = vel_ref(0);

      // std::cout << "Here is the vector v:\n" << vel_ref << std::endl;
      // std::cout << "Here is the vector error\n" << error << std::endl;

      PIDparam.surge.Kp = Kp(1,1);
      PIDparam.surge.Kd = Kd(1,1);
      PIDparam.surge.Ki = Ki(1,1);
      PIDparam.yaw.Kp = Kp(0,0);
      PIDparam.yaw.Kd = Kd(0,0);
      PIDparam.yaw.Ki = Ki(0,0);

      commandVel_pub.publish(velRef);

  }

  //Publish the updated data
  PID_pub.publish(PIDparam);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "directional_PID");
    DirectionalPID directional_PID;

    ros::Rate loop_rate(10);

    while ( ros::ok() )
    {
        directional_PID.SpeedControl();
        ros::spinOnce();
        loop_rate.sleep();
        directional_PID.Timeupdate();
    }

    ROS_INFO("Closing...");
    return 0;

}
