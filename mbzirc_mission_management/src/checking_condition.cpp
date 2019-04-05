#include "ros/ros.h"
#include "utilities.h"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "wmcs_lib/ModemMsgInbox.h"
#include "geometry_msgs/Twist.h"
#include "folaga_msgs/BuoyancyData.h"
#include "folaga_control_system/Task.h"

using namespace folaga_utilities;
using namespace Eigen;
using namespace std;

bool exit_request;

double sub_task_reveived_timestamp;

bool publish_switch;

bool got_pose_info;
bool got_neutral_buoyancy_info;
bool got_floating_buoyancy_info;
bool got_current_buoyancy_info;

bool initialised;
bool checking;

map<string,int> number_of_parameters;

string sub_task_task_type;
VectorXd sub_task_reference;

double current_heading_angle;
double desired_heading_angle;

Vector3d current_lld_position;
Vector3d desired_lld_position;

double current_depth_m;
double desired_depth_m;

int current_buoyancy;
int neutral_buoyancy;
int floating_buoyancy;

std_msgs::String switching_condition_msg;


void subTaskCallback(const folaga_control_system::Task::ConstPtr& msg)
{
  if (initialised){
    if (number_of_parameters.find(msg->task_type) != number_of_parameters.end()){
      sub_task_task_type = msg->task_type;
      sub_task_reference.resize(number_of_parameters[msg->task_type]);
      for (int i = 0; i < number_of_parameters[msg->task_type]; i++)
      {
        sub_task_reference(i) = msg->reference[i];
      }
      checking = true;
      sub_task_reveived_timestamp = ros::Time::now().toSec();
    }
    else
    {
      ROS_ERROR_STREAM("[checking_condition]: sub task unknown");
      ROS_ERROR_STREAM("[checking_condition]: received sub task is " << msg->task_type);
    }
  }
  else
  {
    ROS_ERROR_STREAM("[checking_condition]: sub task called but node is not initialised");
    if (!got_pose_info) ROS_ERROR_STREAM("POSE info is not available");
    if (!got_neutral_buoyancy_info) ROS_ERROR_STREAM("NEUTRAL info is not available");
    if (!got_floating_buoyancy_info) ROS_ERROR_STREAM("FLOATING info is not available");
    if (!got_current_buoyancy_info) ROS_ERROR_STREAM("BUOYANCY info is not available");
  }
}


bool checkMinorThanThreshold(double reference_value, double current_value, double threshold, string type="scalar")
{
  if (type == "scalar")
  {
    return abs(current_value - reference_value) < threshold;
  }
  else if (type == "angular")
  {
    return abs(atan2(sin(current_value - reference_value), cos(current_value - reference_value))) < threshold;
  }
  else
  {
    ROS_ERROR_STREAM("[checking_condition]: unknown type of variable to check");
    ROS_ERROR_STREAM("[checking_condition]: type of variable to check is " << type);
    return false;
  }
}

bool checkMinorThanThreshold(int reference_value, int current_value, int threshold)
{
    return abs(current_value - reference_value) < threshold;
}

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  current_lld_position(0) = msg->linear.x;
  current_lld_position(1) = msg->linear.y;
  current_lld_position(2) = msg->linear.z;

  current_heading_angle = msg->angular.z;

  got_pose_info = true;
}

void getNeutralBuoyancyValueCallback(const folaga_msgs::BuoyancyData::ConstPtr& msg)
{
  neutral_buoyancy = msg->chamberPressure;
  got_neutral_buoyancy_info = true;
}

void getFloatingBuoyancyValueCallback(const folaga_msgs::BuoyancyData::ConstPtr& msg)
{
  floating_buoyancy = msg->chamberPressure;
  got_floating_buoyancy_info = true;
}

void getCurrentBuoyancyValueCallback(const folaga_msgs::BuoyancyData::ConstPtr& msg)
{
  current_buoyancy = msg->chamberPressure;
  got_current_buoyancy_info = true;
}

void modemInboxCallback(const wmcs_lib::ModemMsgInbox::ConstPtr& msg)
{
  if (msg->data == "EXIT"){
    exit_request = true;
  }
}

int main(int argc, char **argv)
{
  exit_request = false;

  sub_task_reveived_timestamp = 0.0;

  publish_switch = false;

  got_pose_info = false;
  got_neutral_buoyancy_info = false;
  got_floating_buoyancy_info = false;
  got_current_buoyancy_info = false;

  initialised = false;
  checking = false;

  sub_task_task_type = "";
  sub_task_reference = Vector3d::Zero();

  current_heading_angle = 0.0;
  desired_heading_angle = 0.0;

  current_lld_position = Vector3d::Zero();
  desired_lld_position = Vector3d::Zero();

  current_depth_m = 0.0;
  desired_depth_m = 0.0;

  current_buoyancy = 0;
  neutral_buoyancy = 0;
  floating_buoyancy = 0;

  ros::init(argc, argv, "checking_condition");
  ros::NodeHandle n;

  number_of_parameters["AUTO_HEADING"] = 1;
  number_of_parameters["AUTO_HEADING_WP"] = 2;
  number_of_parameters["CHANGE_DEPTH"] = 2;
  number_of_parameters["CURVATURE"] = 4;
  number_of_parameters["DISTANCE_MONITOR"] = 2;
  number_of_parameters["GO_FLOATING"] = 0;
  number_of_parameters["GO_LATERAL"] = 3;
  number_of_parameters["GO_NEUTRAL"] = 1;
  number_of_parameters["GO_STRAIGHT"] = 4;
  number_of_parameters["GO_TO_WP"] = 3;
  number_of_parameters["ROTATE"] = 4;
  number_of_parameters["IDLE"] = 0;

  switching_condition_msg.data = "SWITCH";

  // PARAMETER VARIABLES DEFINITION
  double auto_heading_angle;
  double auto_heading_angular_velocity;
  double auto_heading_wp_angle;
  double auto_heading_wp_angular_velocity;
  double change_depth_depth_m;
  double change_depth_depth_speed_m_s;
  double distance_monitor_reactivation_radius_m;
  int go_floating_camera_buoyancy_error;
  int go_neutral_camera_buoyancy_error;
  double go_to_wp_wp_achievement_radius_m;

  // GET PARAMETERS
  n.param<double>("checking_parameters/auto_heading/angle_deg",
                  auto_heading_angle,
                  0.0);
  auto_heading_angle = deg2rad(auto_heading_angle);
  n.param<double>("checking_parameters/auto_heading/angular_velocity_deg_s",
                  auto_heading_angular_velocity,
                  0.0);
  auto_heading_angular_velocity = deg2rad(auto_heading_angular_velocity);

  n.param<double>("checking_parameters/auto_heading_wp/angle_deg",
                  auto_heading_wp_angle,
                  0.0);
  auto_heading_wp_angle = deg2rad(auto_heading_wp_angle);
  n.param<double>("checking_parameters/auto_heading_wp/angular_velocity_deg_s",
                  auto_heading_wp_angular_velocity,
                  0.0);
  auto_heading_wp_angular_velocity = deg2rad(auto_heading_wp_angular_velocity);

  n.param<double>("checking_parameters/change_depth/depth_m",
                  change_depth_depth_m,
                  0.0);
  n.param<double>("checking_parameters/change_depth/depth_speed_m_s",
                  change_depth_depth_speed_m_s,
                  0.0);

  n.param<double>("checking_parameters/distance_monitor/reactivation_radius_m",
                  distance_monitor_reactivation_radius_m,
                  0.0);

  n.param<int>("checking_parameters/go_floating/camera_buoyancy_error",
                  go_floating_camera_buoyancy_error,
                  0);

  n.param<int>("checking_parameters/go_neutral/camera_buoyancy_error",
                  go_neutral_camera_buoyancy_error,
                  0);

  n.param<double>("checking_parameters/go_to_wp/wp_achievement_radius_m",
                  go_to_wp_wp_achievement_radius_m,
                  0.0);

  // PUBISHERS
  ros::Publisher switching_condition_pub = n.advertise<std_msgs::String>("/switching_condition", 1);
  ros::Publisher exit_req_sub = n.advertise<std_msgs::Empty>("/folaga/exit", 1);


  // SUBSCRIBERS
  ros::Subscriber sub_task_sub = n.subscribe("/sub_task", 1, subTaskCallback);

  ros::Subscriber pose_sub = n.subscribe("/folaga/pose", 1, poseCallback);

  ros::Subscriber neutral_buoyancy_sub = n.subscribe("/folaga/neutral", 1, getNeutralBuoyancyValueCallback);
  ros::Subscriber floating_buoyancy_sub = n.subscribe("/folaga/floating", 1, getFloatingBuoyancyValueCallback);
  ros::Subscriber current_buoyancy_sub = n.subscribe("/folaga/balance", 1, getCurrentBuoyancyValueCallback);

  ros::Subscriber modem_inbox_sub = n.subscribe("/Modem/Inbox", 1, modemInboxCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (exit_request)
    {
      exit_request = false;
      std_msgs::Empty exit_msg;
      exit_req_sub.publish(exit_msg);
    }
    if (!initialised)
    {
      if(got_pose_info &
         got_neutral_buoyancy_info &
         got_floating_buoyancy_info &
         got_current_buoyancy_info)
         {
           initialised = true;
         }
    }
    else if (checking)
    {
      if(sub_task_task_type == "IDLE")
      {}
      else if(sub_task_task_type == "AUTO_HEADING")
      {
        publish_switch = checkMinorThanThreshold(
          deg2rad(sub_task_reference(0)),
          current_heading_angle,
          auto_heading_angle,
          "angular");
      }
      else if(sub_task_task_type == "AUTO_HEADING_WP")
      {
        Vector3d target_wp = Vector3d::Zero();
        target_wp(0) = sub_task_reference(0);
        target_wp(1) = sub_task_reference(1);
        publish_switch = checkMinorThanThreshold(
          direction2ll(current_lld_position, target_wp),
          current_heading_angle,
          auto_heading_wp_angle,
          "angular");
      }
      else if(sub_task_task_type == "CHANGE_DEPTH")
      {
        publish_switch = checkMinorThanThreshold(
          sub_task_reference(1),
          current_lld_position(2),
          change_depth_depth_m);
      }
      else if(sub_task_task_type == "CURVATURE")
      {
        publish_switch = !checkMinorThanThreshold(
          sub_task_reveived_timestamp,
          ros::Time::now().toSec(),
          sub_task_reference(3));
      }
      else if(sub_task_task_type == "DISTANCE_MONITOR")
      {
        Vector3d target_wp = Vector3d::Zero();
        target_wp(0) = sub_task_reference(0);
        target_wp(1) = sub_task_reference(1);
        publish_switch = !checkMinorThanThreshold(
          0.0,
          distance2ll(current_lld_position,target_wp),
          distance_monitor_reactivation_radius_m);
      }
      else if(sub_task_task_type == "GO_FLOATING")
      {
        publish_switch = checkMinorThanThreshold(
          floating_buoyancy,
          current_buoyancy,
          go_floating_camera_buoyancy_error);
      }
      else if(sub_task_task_type == "GO_LATERAL")
      {
        publish_switch = !checkMinorThanThreshold(
          sub_task_reveived_timestamp,
          ros::Time::now().toSec(),
          sub_task_reference(2));
      }
      else if(sub_task_task_type == "GO_NEUTRAL")
      {
        publish_switch = checkMinorThanThreshold(
          neutral_buoyancy,
          current_buoyancy,
          go_neutral_camera_buoyancy_error);
      }
      else if(sub_task_task_type == "GO_STRAIGHT")
      {
        publish_switch = !checkMinorThanThreshold(
          sub_task_reveived_timestamp,
          ros::Time::now().toSec(),
          sub_task_reference(3));
      }
      else if(sub_task_task_type == "GO_TO_WP")
      {
        Vector3d target_wp = Vector3d::Zero();
        target_wp(0) = sub_task_reference(0);
        target_wp(1) = sub_task_reference(1);
        publish_switch = checkMinorThanThreshold(
          0.0,
          distance2ll(current_lld_position,target_wp),
          go_to_wp_wp_achievement_radius_m);
      }
      else if(sub_task_task_type == "ROTATE")
      {
        publish_switch = !checkMinorThanThreshold(
          sub_task_reveived_timestamp,
          ros::Time::now().toSec(),
          sub_task_reference(3));
      }

      if (publish_switch){
        switching_condition_pub.publish(switching_condition_msg);
        publish_switch = false;
        checking = false;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
