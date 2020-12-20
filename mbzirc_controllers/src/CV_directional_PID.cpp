#include "ros/ros.h"
#include <math.h>
#include<cstdlib>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PointStamped.h"
#include "mbzirc_controller/directionalPIDparam.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>


class DirectionalPID    {
public:
    DirectionalPID();
    void TgtPosCallback( const geometry_msgs::PointStamped::ConstPtr& target_pos);
    void PIDparamSetCallback( const mbzirc_controller::directionalPIDparam::ConstPtr& K_PID );
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry);
    void TaskIDCallback( const std_msgs::String::ConstPtr& task );
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    Eigen::VectorXd antiWindup(Eigen::VectorXd u, Eigen::VectorXd windupMax);
    void SpeedControl();
    void Timeupdate();
    float VerticalOffset(float dist);

private:
    ros::NodeHandle n = ros::NodeHandle("~");
    ros::Publisher commandVel_pub;

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


    mavros_msgs::PositionTarget             velRef;
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
  commandVel_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local2", 1);
  PID_pub = n.advertise<mbzirc_controller::directionalPIDparam>("PID_param", 1);

  // SUBSCRIBE TO TOPICS
  subtaskSub  = n.subscribe("PIDparamSet", 1, &DirectionalPID::PIDparamSetCallback, this);
  targetPosSub = n.subscribe("/trajectory_fitting/target_pos",1, &DirectionalPID::TgtPosCallback, this);
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
  velRef.coordinate_frame = 8;
  velRef.type_mask = 1479;

  windupMax << 5, 5;

  ////          DA AGGIUSTARE!!!         ////

  derInit = 0;
  searchInit = 1;
  wpCounter = 0;

  dronePos << 0,0,0;
  targetPos << 0,0,0;
  errorOld << 0,0;
  errorI   << 0,0;
  task_id = "IDLE";

	//setting pid params
	Kp(0,0)=0;
	Kd(1,1)=0;

}


void DirectionalPID::TgtPosCallback( const geometry_msgs::PointStamped::ConstPtr& target_pos)  {


  targetPos(0) = target_pos->point.x-0.05; //destra
  targetPos(1) = target_pos->point.y-0.6;  //davanti
  targetPos(2) = target_pos->point.z+0.21; //alto
  //https://www.google.com/search?client=ubuntu&hs=CBF&channel=fs&sxsrf=ACYBGNTZoyHWrkTZY181D77VHoluYugLyg%3A1581669683948&ei=M11GXpO7OaKBi-gPpMmSsAY&q=cv2+has+no+attribute+xfeatures2d&oq=cv2+has+no+attri&gs_l=psy-ab.3.4.0i203l10.9549337.9553300..9556018...0.5..0.93.1331.16......0....1..gws-wiz.......0i71j0i131j0i67j0j35i39j35i39i19.mzIvflDjuqo
  
  std::cerr << "Target Position: \n" << targetPos(0) << " " << targetPos(1) << " " << targetPos(2);


}


void DirectionalPID::PIDparamSetCallback( const mbzirc_controller::directionalPIDparam::ConstPtr& K_PID )  {
    ROS_INFO("Changing Parameters");
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

      double dt;
      if(derInit) dt = secs_fin - secs_start;
      else dt = 0.1;
      secs_start = ros::Time::now().toSec();


      targetPos_NORMALIZED = targetPos.normalized(); // versore drone-target in terna body
      R = droneQuat.normalized().toRotationMatrix(); // matrice di rotazione da terna fissa a body
      xAxis = R * Eigen::Vector3d{1, 0, 0};          //
      yAxis = R * Eigen::Vector3d{0, 1, 0};
      error(0) = - atan2(targetPos(0), targetPos(1)); //errore sullo yaw: 0 è x (dx), 1 è y (avanti)
      error(1) = targetPos.norm()-1.5;   //errore sulla distanza; #TODO: TOGLIERE 1.5
      std::cerr << "The error is:\n" << error << std::endl;


      // Compute errors, errors derivative and integral
      errorD   = (error - errorOld)/(dt)*derInit;
      errorI   = errorI + error*dt;
      if(error(1)>4) errorI << 0,0;
      // || error(1)<1
      errorI   = antiWindup(errorI, windupMax);
      errorOld = error;
      derInit  = 1;

      //MIMO PID
      vel_ref = Kp*error + Kd*errorD + Ki*errorI;

      //Update ros message

      velRef.velocity.x = vel_ref(1)*targetPos_NORMALIZED(0); //dx
      velRef.velocity.y = vel_ref(1)*targetPos_NORMALIZED(1); //avanti
      velRef.velocity.z = vel_ref(1)*(targetPos_NORMALIZED(2)-VerticalOffset(error(1))); //alto
      //se offset POSITIVO, allora lo tiene più in alto nell'inquadratura      
      velRef.yaw_rate = vel_ref(0);

			//command saturation: >0.1 because pixawk doesn't read; upper bound: up to now, not set
			// if(velRef.velocity.x > 0)
			// 		velRef.velocity.x=std::max(0.1,velRef.velocity.x);
			// else if(velRef.velocity.x < 0)
			// 		velRef.velocity.x=std::min(-0.1,velRef.velocity.x);
			
			// if(velRef.velocity.y > 0)
			// 		velRef.velocity.y=std::max(0.1,velRef.velocity.y);
			// else if(velRef.velocity.y < 0)
			// 		velRef.velocity.y=std::min(-0.1,velRef.velocity.y);
			
			// if(velRef.velocity.z > 0)
			// 		velRef.velocity.z=std::max(0.1,velRef.velocity.z);
			// else if(velRef.velocity.z < 0)
			// 		velRef.velocity.z=std::min(-0.1,velRef.velocity.z);
		
		

      std::cerr << "Here is the vector v:\n" << vel_ref << std::endl;

      PIDparam.surge.Kp = Kp(1,1);
      PIDparam.surge.Kd = Kd(1,1);
      PIDparam.surge.Ki = Ki(1,1);
      PIDparam.yaw.Kp = Kp(0,0);
      PIDparam.yaw.Kd = Kd(0,0);
      PIDparam.yaw.Ki = Ki(0,0);

      commandVel_pub.publish(velRef);


  //Publish the updated data
  PID_pub.publish(PIDparam);
}

#define MAXDIST_VOFFSET 20
#define MAX_VOFFSET 1

float DirectionalPID::VerticalOffset(float dist) {

  if(dist>MAXDIST_VOFFSET)
    return MAX_VOFFSET;
  return (float)MAX_VOFFSET*dist/(float)MAXDIST_VOFFSET;
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
